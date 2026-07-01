#ifndef KQ_GAIT_CONTROLLER_H_
#define KQ_GAIT_CONTROLLER_H_

#include "pca9685.h"
#include "oscillator.h"
#include "servo_constants.h"
#include "pca9685_constants.h"
#include "gait_constants.h"
#include "gait_command.h"

#include "board.h"
#include "display/display.h"
#include "mcp_server.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>

#include <atomic>
#include <memory>
#include <cmath>
#include <string>

// Owns the 4-leg gait: the PCA9685 driver, four sine oscillators, the action task + command
// queue, and the self.dog.* MCP tool catalog. MCP callbacks (main thread) only enqueue a
// GaitCommand and return; the action task (own FreeRTOS task, below audio priority) drives the
// servos on a fixed 20ms tick. Stop is cooperative: an atomic flag checked every tick preempts
// the running motion at a keyframe boundary without deleting the task (unlike otto's vTaskDelete).
//
// Lifetime: one instance per board, created by InitializeGaitController() from the board ctor and
// held for the life of the program. Not copyable.
class GaitController
{
private:
    std::shared_ptr<Pca9685> mPca;
    Oscillator mOsc[ServoConstants::LEG_COUNT];   // value member: gait engine, controller lifetime, never escapes
    QueueHandle_t mActionQueue;
    TaskHandle_t  mActionTaskHandle;
    std::atomic<bool> mIsMoving;
    std::atomic<bool> mStopRequested;
    std::atomic<bool> mSafeState;                 // latched on fault: every write is suppressed until a new command clears it
    int mPoseAngle[ServoConstants::LEG_COUNT];    // last commanded angle per channel; slew/ease anchor
    int mFailCount;                               // consecutive I2C write failures -> safe-state

public:
    GaitController(i2c_master_bus_handle_t pcaBus)
    {
        mPca = std::make_shared<Pca9685>(pcaBus);
        mActionTaskHandle = nullptr;
        mFailCount = 0;
        mIsMoving.store(false);
        mStopRequested.store(false);
        mSafeState.store(false);

        const bool tReady = mPca->begin();
        if (!tReady)
        {
            ESP_LOGE(TAG, "PCA9685 init failed; servos held limp");
            mSafeState.store(true);   // a misconfigured chip must not be driven; see enterSafeState
        }

        // Defined-first-pulse obligation: drive every channel to mechanical center before the task
        // starts, so the first servo pulse is never undefined. Skipped when the chip is not ready.
        for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
        {
            mPoseAngle[tCh] = ServoConstants::CENTER_DEGREE;
            if (tReady)
            {
                mPca->setPwm(tCh, Pca9685Constants::SERVO_MID);
            }
        }

        mActionQueue = xQueueCreate(GaitConstants::QUEUE_DEPTH, sizeof(GaitCommand));
        registerMcpTools();
        xTaskCreate(actionTask, "gait_action", GaitConstants::TASK_STACK, this,
                    GaitConstants::TASK_PRIORITY, &mActionTaskHandle);

        // Power-on home runs through the eased path so legs can be mounted aligned to the 90-degree
        // pose with no network/cloud/wake involved.
        enqueue(GaitType::Home, 1);
    }

    GaitController(const GaitController&) = delete;
    GaitController& operator=(const GaitController&) = delete;

private:
    static constexpr const char* TAG = "GaitController";

    // Piecewise-linear angle->count through the three named endpoints so 90 degrees maps exactly to
    // SERVO_MID (the center home() commands), keeping gait-neutral and home identical.
    static int angleToCount(int angleDeg)
    {
        int tAngle = clampAngle(angleDeg);
        if (tAngle <= ServoConstants::CENTER_DEGREE)
        {
            return Pca9685Constants::SERVO_MIN
                 + (Pca9685Constants::SERVO_MID - Pca9685Constants::SERVO_MIN)
                 * (tAngle - ServoConstants::MIN_DEGREE)
                 / (ServoConstants::CENTER_DEGREE - ServoConstants::MIN_DEGREE);
        }
        return Pca9685Constants::SERVO_MID
             + (Pca9685Constants::SERVO_MAX - Pca9685Constants::SERVO_MID)
             * (tAngle - ServoConstants::CENTER_DEGREE)
             / (ServoConstants::MAX_DEGREE - ServoConstants::CENTER_DEGREE);
    }

    static int clampAngle(int angleDeg)
    {
        if (angleDeg < ServoConstants::MIN_DEGREE)
        {
            return ServoConstants::MIN_DEGREE;
        }
        if (angleDeg > ServoConstants::MAX_DEGREE)
        {
            return ServoConstants::MAX_DEGREE;
        }
        return angleDeg;
    }

    // Slew-limited write: caps per-tick angle change to bound di/dt (5V brownout), updates the pose
    // anchor, and escalates to the limp safe-state after I2C_FAIL_LIMIT consecutive bus failures.
    void writeChannel(int channel, int angleDeg)
    {
        if (mSafeState.load())
        {
            return;   // latched limp: never re-energize a servo until a new command clears the latch
        }
        int tTarget = clampAngle(angleDeg);
        const int tDelta = tTarget - mPoseAngle[channel];
        if (tDelta > GaitConstants::SLEW_DEG_PER_TICK)
        {
            tTarget = mPoseAngle[channel] + GaitConstants::SLEW_DEG_PER_TICK;
        }
        else if (tDelta < -GaitConstants::SLEW_DEG_PER_TICK)
        {
            tTarget = mPoseAngle[channel] - GaitConstants::SLEW_DEG_PER_TICK;
        }
        mPoseAngle[channel] = tTarget;

        if (mPca->setPwm(channel, angleToCount(tTarget)))
        {
            mFailCount = 0;
        }
        else
        {
            mFailCount++;
            if (mFailCount >= GaitConstants::I2C_FAIL_LIMIT)
            {
                enterSafeState();
            }
        }
    }

    // Cosine ease (zero velocity at both ends) from the current pose to target over `steps` ticks.
    // Returns false if interrupted by a stop request (only when interruptible).
    bool easeToPose(const int target[ServoConstants::LEG_COUNT], int steps, bool interruptible)
    {
        int tStart[ServoConstants::LEG_COUNT];
        for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
        {
            tStart[tCh] = mPoseAngle[tCh];
        }
        for (int tStep = 1; tStep <= steps; tStep++)
        {
            if (mSafeState.load())
            {
                return false;   // fault latched mid-ease: stop driving
            }
            if (interruptible && mStopRequested.load())
            {
                return false;
            }
            const double tF = 0.5 * (1.0 - std::cos(GaitConstants::PI * tStep / steps));
            for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
            {
                const int tAngle = tStart[tCh] + (int)std::lround((target[tCh] - tStart[tCh]) * tF);
                writeChannel(tCh, tAngle);
            }
            vTaskDelay(pdMS_TO_TICKS(GaitConstants::TICK_MS));
        }
        return true;
    }

    // Continuous sine gait for `cycles` periods, then an uninterruptible ease back to center so the
    // legs always end at a defined pose (even if a stop broke the cycle loop early).
    void runContinuous(int amplitudeDeg, int periodMs, const double phase[ServoConstants::LEG_COUNT],
                       bool applyReverse, int cycles)
    {
        for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
        {
            const bool tReverse = applyReverse ? ServoConstants::REVERSE[tCh] : false;
            mOsc[tCh].configure(amplitudeDeg, periodMs, phase[tCh], tReverse, GaitConstants::TICK_MS);
        }
        const int tTicks = cycles * periodMs / GaitConstants::TICK_MS;
        for (int tTick = 0; tTick < tTicks; tTick++)
        {
            if (mStopRequested.load() || mSafeState.load())
            {
                break;
            }
            for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
            {
                writeChannel(tCh, mOsc[tCh].currentAngle());
                mOsc[tCh].advance();
            }
            vTaskDelay(pdMS_TO_TICKS(GaitConstants::TICK_MS));
        }
        easeToPose(GaitConstants::POSE_HOME, GaitConstants::EASE_STEPS, false);
    }

    // Front-right leg salute; the other three legs hold center for static stability.
    void runWave()
    {
        int tDown[ServoConstants::LEG_COUNT] = { ServoConstants::CENTER_DEGREE, GaitConstants::WAVE_LOW_DEG,
                                                 ServoConstants::CENTER_DEGREE, ServoConstants::CENTER_DEGREE };
        int tUp[ServoConstants::LEG_COUNT]   = { ServoConstants::CENTER_DEGREE, GaitConstants::WAVE_HIGH_DEG,
                                                 ServoConstants::CENTER_DEGREE, ServoConstants::CENTER_DEGREE };
        if (!easeToPose(tDown, GaitConstants::WAVE_SEG_STEPS, true))
        {
            return;
        }
        for (int tRep = 0; tRep < GaitConstants::WAVE_REPEATS; tRep++)
        {
            if (!easeToPose(tUp, GaitConstants::WAVE_SEG_STEPS, true))
            {
                return;
            }
            if (!easeToPose(tDown, GaitConstants::WAVE_SEG_STEPS, true))
            {
                return;
            }
        }
        easeToPose(GaitConstants::POSE_HOME, GaitConstants::WAVE_SEG_STEPS, true);
    }

    void runCommand(GaitCommand command)   // transient command value copied from the queue, not retained
    {
        switch (command.type)
        {
            case GaitType::Home:
                easeToPose(GaitConstants::POSE_HOME, GaitConstants::EASE_STEPS, true);
                break;
            case GaitType::Sit:
                easeToPose(GaitConstants::POSE_SIT, GaitConstants::EASE_STEPS, true);
                break;
            case GaitType::Stretch:
                if (easeToPose(GaitConstants::POSE_STRETCH, GaitConstants::EASE_STEPS, true))
                {
                    holdPose(GaitConstants::STRETCH_HOLD_MS);
                    easeToPose(GaitConstants::POSE_HOME, GaitConstants::EASE_STEPS, true);
                }
                break;
            case GaitType::Wave:
                runWave();
                break;
            case GaitType::WalkForward:
                runContinuous(GaitConstants::WALK_AMP_DEG, GaitConstants::WALK_PERIOD_MS,
                              GaitConstants::WALK_FORWARD_PHASE, true, command.steps);
                break;
            case GaitType::WalkBack:
                runContinuous(GaitConstants::WALK_AMP_DEG, GaitConstants::WALK_PERIOD_MS,
                              GaitConstants::WALK_BACK_PHASE, true, command.steps);
                break;
            case GaitType::TurnLeft:
                runContinuous(GaitConstants::TURN_AMP_DEG, GaitConstants::TURN_PERIOD_MS,
                              GaitConstants::TURN_LEFT_PHASE, false, command.steps);
                break;
            case GaitType::TurnRight:
                runContinuous(GaitConstants::TURN_AMP_DEG, GaitConstants::TURN_PERIOD_MS,
                              GaitConstants::TURN_RIGHT_PHASE, false, command.steps);
                break;
            case GaitType::Dance:
                runContinuous(GaitConstants::DANCE_AMP_DEG, GaitConstants::DANCE_PERIOD_MS,
                              GaitConstants::DANCE_PHASE, true, GaitConstants::DANCE_CYCLES);
                break;
        }
    }

    // Stop-checked dwell so a held pose (stretch) still preempts within one tick.
    void holdPose(int durationMs)
    {
        const int tTicks = durationMs / GaitConstants::TICK_MS;
        for (int tTick = 0; tTick < tTicks; tTick++)
        {
            if (mStopRequested.load() || mSafeState.load())
            {
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(GaitConstants::TICK_MS));
        }
    }

    // Drop-oldest enqueue (depth 1~2) so a stale move never delays a fresh command. Never blocks.
    void enqueue(GaitType type, int steps)
    {
        GaitCommand tCommand;
        tCommand.type = type;
        tCommand.steps = steps;
        if (uxQueueMessagesWaiting(mActionQueue) >= GaitConstants::QUEUE_DEPTH)
        {
            GaitCommand tDropped;
            xQueueReceive(mActionQueue, &tDropped, 0);
        }
        xQueueSend(mActionQueue, &tCommand, 0);
    }

    // Preempt the running motion (atomic flag, checked every tick), flush pending moves, then queue
    // a Home so the robot returns to center even if it was idle. Flag is set before the reset so the
    // gait task observes it (single producer = MCP callback, single consumer = gait task).
    void requestStop()
    {
        mStopRequested.store(true);
        xQueueReset(mActionQueue);
        enqueue(GaitType::Home, 1);
    }

    // Fault path: cut every servo's pulse train (full-OFF) so the robot goes limp, not wedged. Latches
    // mSafeState so the in-flight motion cannot re-energize a channel (setPwm clears the full-OFF bit);
    // the latch is cleared only when a fresh command is dequeued (actionTask), giving transient bus
    // faults one retry while a persistent fault keeps the robot limp.
    // Only the I2C write-failure path drives this; disconnect->limp and brownout triggers would require
    // the network/protocol layer and are not wired by this board.
    void enterSafeState()
    {
        mSafeState.store(true);
        for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
        {
            mPca->fullOff(tCh);
        }
        mFailCount = 0;
        mIsMoving.store(false);
        ESP_LOGE(TAG, "entered safe-state: all servos limp");
    }

    static void actionTask(void* arg)
    {
        GaitController* tSelf = static_cast<GaitController*>(arg);
        GaitCommand tCommand;
        for (;;)
        {
            if (xQueueReceive(tSelf->mActionQueue, &tCommand, pdMS_TO_TICKS(GaitConstants::QUEUE_RECEIVE_MS)) == pdTRUE)
            {
                tSelf->mStopRequested.store(false);
                tSelf->mFailCount = 0;
                if (tSelf->mSafeState.exchange(false))
                {
                    // Recovering from a latched fault: the servos went limp and may have drooped to an
                    // unknown position (SG90 has no feedback), so the stale per-channel anchor is wrong.
                    // Re-anchor to mechanical center; the first re-energizing write is necessarily blind,
                    // but subsequent ticks are slew-bounded from a defined assumption.
                    for (int tCh = 0; tCh < ServoConstants::LEG_COUNT; tCh++)
                    {
                        tSelf->mPoseAngle[tCh] = ServoConstants::CENTER_DEGREE;
                    }
                }
                tSelf->mIsMoving.store(true);
                tSelf->runCommand(tCommand);
                tSelf->mIsMoving.store(false);
                vTaskDelay(pdMS_TO_TICKS(GaitConstants::TICK_MS));
            }
        }
    }

    // Catalog of self.dog.* tools on the framework McpServer singleton. Callbacks enqueue and return
    // immediately (narrow lambda exception: McpServer::AddTool requires a std::function callable).
    void registerMcpTools()
    {
        auto& tMcp = McpServer::GetInstance();

        tMcp.AddTool("self.dog.walk_forward",
                     "앞으로 느리게 미끄러지듯 이동(마찰 셔플, 걸음당 약 1cm, 바닥 의존). 확실히 움직이려면 제자리 회전 권장",
                     PropertyList({ Property("steps", kPropertyTypeInteger,
                                             GaitConstants::STEP_DEFAULT, GaitConstants::STEP_MIN, GaitConstants::STEP_MAX) }),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::WalkForward, properties["steps"].value<int>());
                         return true;
                     });

        tMcp.AddTool("self.dog.walk_back",
                     "뒤로 느리게 미끄러지듯 이동(마찰 셔플, best-effort)",
                     PropertyList({ Property("steps", kPropertyTypeInteger,
                                             GaitConstants::STEP_DEFAULT, GaitConstants::STEP_MIN, GaitConstants::STEP_MAX) }),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::WalkBack, properties["steps"].value<int>());
                         return true;
                     });

        tMcp.AddTool("self.dog.turn_left",
                     "제자리에서 왼쪽으로 회전(신뢰 동작)",
                     PropertyList({ Property("steps", kPropertyTypeInteger,
                                             GaitConstants::STEP_DEFAULT, GaitConstants::STEP_MIN, GaitConstants::STEP_MAX) }),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::TurnLeft, properties["steps"].value<int>());
                         return true;
                     });

        tMcp.AddTool("self.dog.turn_right",
                     "제자리에서 오른쪽으로 회전(신뢰 동작)",
                     PropertyList({ Property("steps", kPropertyTypeInteger,
                                             GaitConstants::STEP_DEFAULT, GaitConstants::STEP_MIN, GaitConstants::STEP_MAX) }),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::TurnRight, properties["steps"].value<int>());
                         return true;
                     });

        tMcp.AddTool("self.dog.dance", "제자리에서 춤춘다", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::Dance, 1);
                         return true;
                     });

        tMcp.AddTool("self.dog.wave", "앞다리를 들어 인사한다(제자리)", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::Wave, 1);
                         return true;
                     });

        tMcp.AddTool("self.dog.sit", "앉는 자세를 취한다", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::Sit, 1);
                         return true;
                     });

        tMcp.AddTool("self.dog.stretch", "기지개 자세를 취한 뒤 복귀한다", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         enqueue(GaitType::Stretch, 1);
                         return true;
                     });

        tMcp.AddTool("self.dog.stop", "즉시 멈추고 홈(중심) 자세로 복귀한다", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         requestStop();
                         return true;
                     });

        tMcp.AddTool("self.dog.home", "전 채널을 중심(90도)으로 복귀시킨다(다리 정렬 장착용, 진행 동작 선점)", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         requestStop();   // home preempts like stop: flush the queue and re-home
                         return true;
                     });

        tMcp.AddTool("self.dog.set_emotion",
                     "OLED 표정을 바꾼다(happy/sad/angry/neutral 등)",
                     PropertyList({ Property("emotion", kPropertyTypeString, std::string("neutral")) }),
                     [](const PropertyList& properties) -> ReturnValue {
                         std::string tEmotion = properties["emotion"].value<std::string>();
                         Display* tDisplay = Board::GetInstance().GetDisplay();
                         if (tDisplay != nullptr)
                         {
                             tDisplay->SetEmotion(tEmotion.c_str());
                         }
                         return true;
                     });

        tMcp.AddTool("self.dog.get_status", "로봇 상태를 반환한다: moving 또는 idle", PropertyList(),
                     [this](const PropertyList& properties) -> ReturnValue {
                         return mIsMoving.load()
                                    ? std::string("moving") : std::string("idle");
                     });
    }
};

// Create the single GaitController for this board. Idempotent: the instance is held for the life of
// the program.
inline void InitializeGaitController(i2c_master_bus_handle_t pcaBus)
{
    static std::shared_ptr<GaitController> tInstance;
    if (!tInstance)
    {
        tInstance = std::make_shared<GaitController>(pcaBus);
    }
}

#endif // KQ_GAIT_CONTROLLER_H_
