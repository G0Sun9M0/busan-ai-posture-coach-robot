// korean-quadruped board: WifiBoard derivative that brings up audio (NoAudioCodecSimplex), the
// SSD1306 OLED on I2C0, and the PCA9685 servo driver on a separate I2C1, then hands the PCA9685 bus
// to the GaitController (self.dog.* MCP tools). Two independent I2C controllers keep OLED refresh
// and servo writes from ever contending. No app button: the English WakeNet is the sole runtime
// trigger; WiFi provisioning falls back to the framework auto-AP when NVS has no credentials.
//
// 바른자세 코치: MPU-9250 기울기 센서를 서보(PCA9685)와 같은 I2C1 버스(GPIO1/2)에 직접(유선) 연결(주소 0x68).
// 로봇이 직접 센서를 읽어 앞으로 숙인 각도(pitch)를 계산하고, "거북목"이면 WakeWordInvoke로
// "자세가 거북목이야"를 주입 -> 클라우드가 프롬프트대로 음성 코칭 + stretch + angry 표정.

#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "display/display.h"
#include "led/single_led.h"
#include "config.h"
#include "gait_controller.h"

#include <driver/i2c_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>

#include "application.h"
#include "mcp_server.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "KoreanQuadruped"

// ---- 기울기 센서(MPU-9250, I2C0 0x68) 직결 자세 감지 ----
#define MPU_ADDR             0x68
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_ACCEL_XOUT   0x3B
#define POSTURE_WARN_DEG     15.0f                 // 이 각도 이상이면 약간 숙임
#define POSTURE_BAD_DEG      30.0f                 // 이 각도 이상 숙이면 거북목
#define POSTURE_COOLDOWN_US  (20LL * 1000000)      // 같은 경고 최소 20초 간격

static i2c_master_dev_handle_t s_mpu = nullptr;
static volatile bool s_studyMode = false;   // "공부 시작할게" 로 켜짐. 켜졌을 때만 거북목 경고.

// MPU를 0.5초마다 읽어 거북목이면 로봇 자동 반응을 트리거하는 태스크.
static void posture_task(void* arg)
{
    int64_t lastTrigger = -POSTURE_COOLDOWN_US;
    uint8_t reg = MPU_REG_ACCEL_XOUT;
    uint8_t raw[6];
    int failCount = 0;
    bool wasBad = false;   // 거북목 상태였는지(바른자세 복귀 시 표정 원복용)
    while (true) {
        if (i2c_master_transmit_receive(s_mpu, &reg, 1, raw, sizeof(raw),
                                        pdMS_TO_TICKS(1000)) != ESP_OK) {
            if (failCount++ % 4 == 0)   // 2초마다 한 번만 경고
                ESP_LOGW(TAG, "[자세] MPU 응답 없음 — 배선 확인 (SDA=GPIO1/SCL=GPIO2/3V3/GND/AD0=GND)");
            uint8_t tWake[2] = { MPU_REG_PWR_MGMT_1, 0x00 };   // 재연결 대비 깨우기 재시도
            i2c_master_transmit(s_mpu, tWake, sizeof(tWake), pdMS_TO_TICKS(200));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        failCount = 0;
        {
            int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
            int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
            int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
            float gx = ax / 16384.0f, gy = ay / 16384.0f, gz = az / 16384.0f;
            // 앞으로 숙인 각도. 장착 방향에 따라 축이 다르면 gx를 gy로 바꾸세요.
            float pitch = atan2f(gx, sqrtf(gy * gy + gz * gz)) * 180.0f / (float)M_PI;
            float tilt = fabsf(pitch);
            const char* verdict = (tilt >= POSTURE_BAD_DEG)  ? "거북목"
                                : (tilt >= POSTURE_WARN_DEG) ? "약간 숙임"
                                                             : "바른자세";

            // 센서값 실시간 로그 (AI 파이프라인 확인용) — 0.5초마다 출력
            ESP_LOGI(TAG, "[자세] pitch=%6.1f도  ->  %s  %s", pitch, verdict,
                     s_studyMode ? "[공부중]" : "[대기]");

            Display* display = Board::GetInstance().GetDisplay();
            if (tilt >= POSTURE_BAD_DEG && s_studyMode) {     // 공부 모드일 때만 거북목 경고
                // ★ 표정 강제: 거북목인 동안 계속 angry (0.5초마다 재설정 → AI가 무표정으로 둬도 덮어씀)
                if (display) display->SetEmotion("angry");
                wasBad = true;
                int64_t now = esp_timer_get_time();
                int64_t since = now - lastTrigger;
                if (since >= POSTURE_COOLDOWN_US) {
                    lastTrigger = now;
                    const char* msg = "자세가 거북목이야";
                    ESP_LOGW(TAG, "[AI 전송] 거북목! -> WakeWordInvoke(\"%s\")  (AI가 이 문장을 받습니다)", msg);
                    Application::GetInstance().Schedule([msg]() {
                        Application::GetInstance().WakeWordInvoke(msg);
                    });
                } else {
                    ESP_LOGI(TAG, "[쿨다운] 거북목 지속 — 다음 AI 전송까지 %lld초",
                             (long long)((POSTURE_COOLDOWN_US - since) / 1000000));
                }
            } else if (tilt < POSTURE_WARN_DEG && wasBad) {   // 거북목 → 바른자세 복귀
                if (display) display->SetEmotion("happy");     // 자세 교정하면 칭찬 표정
                wasBad = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

class KoreanQuadrupedBoard : public WifiBoard
{
private:
    i2c_master_bus_handle_t mDisplayI2cBus;   // I2C0, OLED 0x3C only
    i2c_master_bus_handle_t mPca9685I2cBus;   // I2C1, PCA9685 0x40 + MPU-9250 0x68
    std::shared_ptr<Display> mDisplay;
    esp_lcd_panel_io_handle_t mPanelIo;
    esp_lcd_panel_handle_t mPanel;

    static i2c_master_bus_handle_t createI2cBus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
    {
        i2c_master_bus_config_t tConfig = {};
        tConfig.i2c_port = port;
        tConfig.sda_io_num = sda;
        tConfig.scl_io_num = scl;
        tConfig.clk_source = I2C_CLK_SRC_DEFAULT;
        tConfig.glitch_ignore_cnt = I2C_GLITCH_IGNORE_CNT;
        tConfig.intr_priority = 0;
        tConfig.trans_queue_depth = 0;
        tConfig.flags.enable_internal_pullup = 1;
        i2c_master_bus_handle_t tBus = nullptr;
        ESP_ERROR_CHECK(i2c_new_master_bus(&tConfig, &tBus));
        return tBus;
    }

    // MPU-9250을 OLED와 같은 I2C0 버스에 등록하고 깨운 뒤 자세 감지 태스크를 띄운다.
    void initializeMpu()
    {
        i2c_device_config_t tMpuCfg = {};
        tMpuCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        tMpuCfg.device_address = MPU_ADDR;
        tMpuCfg.scl_speed_hz = 400000;
        // 서보(PCA9685)와 같은 I2C1 버스 공유 (SDA=GPIO1, SCL=GPIO2, MPU 0x68 vs 서보 0x40)
        if (i2c_master_bus_add_device(mPca9685I2cBus, &tMpuCfg, &s_mpu) != ESP_OK) {
            ESP_LOGE(TAG, "MPU-9250 등록 실패 - I2C 배선(SDA=GPIO1/SCL=GPIO2/3V3/GND) 확인");
            return;
        }
        uint8_t tWake[2] = { MPU_REG_PWR_MGMT_1, 0x00 };   // 절전 해제(실패해도 태스크는 시작 — 빵판 접촉 불량 대비 재시도)
        i2c_master_transmit(s_mpu, tWake, sizeof(tWake), pdMS_TO_TICKS(1000));
        xTaskCreate(posture_task, "posture", 4096, nullptr, 4, nullptr);
        ESP_LOGI(TAG, "MPU 자세 감지 태스크 시작 — 연결되면 [자세] 로그가 뜹니다");
    }

    // "공부 시작할게"/"공부 끝" 음성 의도로 자세 감시를 켜고 끄는 MCP 도구.
    void registerStudyMcpTools()
    {
        auto& tMcp = McpServer::GetInstance();
        tMcp.AddTool("self.study.start",
                     "공부 모드 시작 — 자세 감시를 켠다. 사용자가 '공부하자/공부 시작할게/이제 공부' 처럼 공부를 시작한다고 하면 반드시 이 도구를 호출한다.",
                     PropertyList(),
                     [](const PropertyList&) -> ReturnValue {
                         s_studyMode = true;
                         ESP_LOGI(TAG, "[공부모드] ON — 자세 감시 시작");
                         return std::string("공부 모드 시작. 자세를 지켜볼게요.");
                     });
        tMcp.AddTool("self.study.stop",
                     "공부 모드 종료 — 자세 감시를 끈다. 사용자가 '공부 끝/그만할래/공부 끝났어' 처럼 공부를 마친다고 하면 반드시 이 도구를 호출한다.",
                     PropertyList(),
                     [](const PropertyList&) -> ReturnValue {
                         s_studyMode = false;
                         ESP_LOGI(TAG, "[공부모드] OFF — 자세 감시 종료");
                         return std::string("공부 끝! 수고했어요.");
                     });
    }

    void initializeSsd1306()
    {
        esp_lcd_panel_io_i2c_config_t tIoConfig = {};
        tIoConfig.dev_addr = DISPLAY_I2C_ADDR;
        tIoConfig.control_phase_bytes = 1;
        tIoConfig.dc_bit_offset = 6;
        tIoConfig.lcd_cmd_bits = 8;
        tIoConfig.lcd_param_bits = 8;
        tIoConfig.scl_speed_hz = DISPLAY_I2C_SPEED_HZ;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(mDisplayI2cBus, &tIoConfig, &mPanelIo));

        esp_lcd_panel_dev_config_t tPanelConfig = {};
        tPanelConfig.reset_gpio_num = -1;
        tPanelConfig.bits_per_pixel = 1;
        esp_lcd_panel_ssd1306_config_t tSsd1306Config = {};
        tSsd1306Config.height = static_cast<uint8_t>(DISPLAY_HEIGHT);
        tPanelConfig.vendor_config = &tSsd1306Config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(mPanelIo, &tPanelConfig, &mPanel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(mPanel));
        if (esp_lcd_panel_init(mPanel) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize SSD1306");
            mDisplay = std::make_shared<NoDisplay>();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(mPanel, true));
        mDisplay = std::make_shared<OledDisplay>(mPanelIo, mPanel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                                 DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

public:
    KoreanQuadrupedBoard()
    {
        mPanelIo = nullptr;
        mPanel = nullptr;
        mDisplayI2cBus = createI2cBus(DISPLAY_I2C_PORT, DISPLAY_I2C_SDA_PIN, DISPLAY_I2C_SCL_PIN);
        mPca9685I2cBus = createI2cBus(Pca9685Constants::I2C_PORT, Pca9685Constants::SDA_GPIO,
                                      Pca9685Constants::SCL_GPIO);
        initializeSsd1306();
        InitializeGaitController(mPca9685I2cBus);
        initializeMpu();                       // 기울기 센서: 서보 I2C1 버스(GPIO1/2) 공유
        registerStudyMcpTools();               // "공부 시작할게" → 자세 감시 ON
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static NoAudioCodecSimplex audioCodec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audioCodec;
    }

    virtual Display* GetDisplay() override
    {
        return mDisplay.get();
    }

    virtual Led* GetLed() override
    {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }
};

DECLARE_BOARD(KoreanQuadrupedBoard);
