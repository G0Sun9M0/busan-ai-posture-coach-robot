#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "esp_err.h"

/*
    05_mic_test

    INMP441 I2S 마이크 단독 테스트 코드

    연결:
    INMP441 WS   -> ESP32-S3 GPIO4
    INMP441 SCK  -> ESP32-S3 GPIO5
    INMP441 SD   -> ESP32-S3 GPIO6
    INMP441 VDD  -> 3.3V
    INMP441 GND  -> GND
    INMP441 L/R  -> GND

    L/R을 GND에 연결했으므로 LEFT 채널로 읽습니다.

    동작:
    시리얼 모니터에 마이크 입력 크기 출력
*/

#define I2S_PORT                I2S_NUM_0

#define I2S_MIC_WS_GPIO         GPIO_NUM_4
#define I2S_MIC_SCK_GPIO        GPIO_NUM_5
#define I2S_MIC_SD_GPIO         GPIO_NUM_6

#define SAMPLE_RATE             16000
#define READ_SAMPLE_COUNT       512

static i2s_chan_handle_t rx_chan = NULL;

static void i2s_mic_init(void)
{
    printf("Initializing I2S microphone input...\n");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    /*
        INMP441은 보통 24bit 데이터를 32bit 슬롯에 실어서 보냅니다.
        그래서 32bit로 읽고, 계산할 때 일부 비트를 정리해서 사용합니다.

        L/R 핀이 GND이므로 LEFT 채널을 읽습니다.
    */
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_MONO
    );

    slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_MIC_SCK_GPIO,
            .ws   = I2S_MIC_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_MIC_SD_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    printf("I2S microphone initialized.\n");
    printf("WS  = GPIO%d\n", I2S_MIC_WS_GPIO);
    printf("SCK = GPIO%d\n", I2S_MIC_SCK_GPIO);
    printf("SD  = GPIO%d\n", I2S_MIC_SD_GPIO);
    printf("L/R = GND, so reading LEFT channel\n");
}

static int calculate_mic_level(int32_t *samples, int sample_count)
{
    int64_t sum_abs = 0;
    int32_t max_abs = 0;

    for (int i = 0; i < sample_count; i++)
    {
        /*
            INMP441의 유효 데이터는 32bit 안에 들어옵니다.
            너무 큰 raw 값을 그대로 쓰지 않기 위해 8bit 정도 오른쪽으로 이동합니다.
        */
        int32_t sample = samples[i] >> 8;

        if (sample < 0)
        {
            sample = -sample;
        }

        sum_abs += sample;

        if (sample > max_abs)
        {
            max_abs = sample;
        }
    }

    int avg_abs = (int)(sum_abs / sample_count);

    /*
        숫자를 보기 쉽게 줄입니다.
        필요하면 이 나누는 값을 조정해도 됩니다.
    */
    int level = avg_abs / 1000;

    return level;
}

static void print_level_bar(int level)
{
    int bar_count = level / 5;

    if (bar_count > 40)
    {
        bar_count = 40;
    }

    printf("MIC LEVEL: %4d | ", level);

    for (int i = 0; i < bar_count; i++)
    {
        printf("#");
    }

    printf("\n");
}

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("05 MIC TEST START\n");
    printf("INMP441 I2S MIC TEST\n");
    printf("WS  -> GPIO4\n");
    printf("SCK -> GPIO5\n");
    printf("SD  -> GPIO6\n");
    printf("VDD -> 3.3V\n");
    printf("GND -> GND\n");
    printf("L/R -> GND, LEFT channel\n");
    printf("====================================\n");

    i2s_mic_init();

    int32_t samples[READ_SAMPLE_COUNT];
    size_t bytes_read = 0;

    while (1)
    {
        esp_err_t ret = i2s_channel_read(
            rx_chan,
            samples,
            sizeof(samples),
            &bytes_read,
            pdMS_TO_TICKS(1000)
        );

        if (ret != ESP_OK)
        {
            printf("I2S read failed. err=0x%x\n", ret);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int sample_count = bytes_read / sizeof(int32_t);

        if (sample_count <= 0)
        {
            printf("No microphone samples read.\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int level = calculate_mic_level(samples, sample_count);

        print_level_bar(level);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}