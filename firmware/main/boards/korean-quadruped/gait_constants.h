#ifndef KQ_GAIT_CONSTANTS_H_
#define KQ_GAIT_CONSTANTS_H_

#include <freertos/FreeRTOS.h>
#include <stdint.h>
#include <cmath>

// Gait engine timing, queue policy, and per-motion parameters for the
// 1-DOF-per-leg waddle. 1-DOF legs cannot lift, so turn-in-place / dance / wave
// / sit / stretch are the reliable motions and walk_forward/walk_back are an
// explicit best-effort friction shuffle (no leg lift, ~1cm/step, floor-dependent).
// The tick is locked to the SG90 50Hz frame: raising it above 50Hz does not make
// motion smoother (the servo holds the same pulse until the next frame).

namespace GaitConstants
{
    static constexpr TickType_t TICK_MS       = 20;        // SG90 update window (50Hz frame)
    static constexpr UBaseType_t QUEUE_DEPTH  = 2;         // 1~2: drop-oldest beyond this
    static constexpr uint32_t TASK_STACK      = 1024 * 3;
    static constexpr UBaseType_t TASK_PRIORITY = 3;        // below audio (prio 8); must not starve I2S/wake
    static constexpr int QUEUE_RECEIVE_MS     = 1000;      // task wakes to re-check stop flag even when idle
    static constexpr int I2C_FAIL_LIMIT       = 5;         // consecutive write failures -> enter safe-state

    // Continuous gait (sine oscillator) parameters.
    static constexpr int WALK_AMP_DEG    = 25;
    static constexpr int WALK_PERIOD_MS  = 1200;
    static constexpr int TURN_AMP_DEG    = 30;
    static constexpr int TURN_PERIOD_MS  = 900;
    static constexpr int DANCE_AMP_DEG   = 28;             // sized so peak velocity stays at/under SLEW_DEG_PER_TICK
    static constexpr int DANCE_PERIOD_MS = 600;
    static constexpr int DANCE_CYCLES    = 7;              // ~4.2s at 600ms/cycle

    // Per-leg phase offset (radians), indexed FL,FR,RL,RR. Diagonal pairs (FL+RR, FR+RL)
    // move antiphase for the 1-DOF trot; turns oppose left/right phase. These are bench-tunable
    // starting values: the sine output is symmetric, so forward thrust comes from foot-floor
    // friction, not waveform asymmetry. REVERSE (servo_constants.h) is applied for
    // walk/dance only; turns set reverse=false (left/right phase opposition draws the arc instead).
    static constexpr double PI = M_PI;
    static constexpr double WALK_FORWARD_PHASE[4] = { 0.0, PI,  PI,  0.0 };
    static constexpr double WALK_BACK_PHASE[4]    = { PI,  0.0, 0.0, PI  };
    static constexpr double TURN_LEFT_PHASE[4]    = { 0.0, PI,  0.0, PI  };
    static constexpr double TURN_RIGHT_PHASE[4]   = { PI,  0.0, PI,  0.0 };
    static constexpr double DANCE_PHASE[4]        = { 0.0, PI / 2.0, PI, 3.0 * PI / 2.0 };

    // Per-tick slew limit (degrees per 20ms tick) to cap di/dt and avoid 5V brownout.
    static constexpr int SLEW_DEG_PER_TICK = 6;

    // Discrete keyframe pose ramp resolution (number of 20ms steps; ~600ms).
    static constexpr int EASE_STEPS = 30;

    // Wave gesture: front-right leg sweeps between these angles, repeated.
    static constexpr int WAVE_LOW_DEG   = 40;
    static constexpr int WAVE_HIGH_DEG  = 70;
    static constexpr int WAVE_REPEATS   = 3;
    static constexpr int WAVE_SEG_STEPS = 12;   // 20ms steps per wave up/down ramp (~240ms)

    // Pose targets (per leg FL,FR,RL,RR), absolute degrees.
    static constexpr int POSE_HOME[4]    = { 90, 90, 90, 90 };
    static constexpr int POSE_SIT[4]     = { 60, 120, 60, 120 };
    static constexpr int POSE_STRETCH[4] = { 130, 50, 50, 130 };
    static constexpr int STRETCH_HOLD_MS = 800;

    // Step count parameter bounds for walk/turn tools.
    static constexpr int STEP_DEFAULT = 4;
    static constexpr int STEP_MIN     = 1;
    static constexpr int STEP_MAX     = 20;
}

#endif // KQ_GAIT_CONSTANTS_H_
