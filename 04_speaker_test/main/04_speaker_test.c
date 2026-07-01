#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "esp_err.h"

/*
    04_speaker_test

    MAX98357A I2S 앰프 테스트 코드

    연결:
    MAX98357A DIN  -> ESP32-S3 GPIO7
    MAX98357A BCLK -> ESP32-S3 GPIO15
    MAX98357A LRC  -> ESP32-S3 GPIO16
    MAX98357A Vin  -> 5V
    MAX98357A GND  -> GND
    스피커 + / -   -> 8Ω 스피커

    동작:
    삐-삐-삐 소리를 반복 출력
*/

#define I2S_PORT                I2S_NUM_0

#define I2S_DOUT_GPIO           GPIO_NUM_7
#define I2S_BCLK_GPIO           GPIO_NUM_15
#define I2S_LRC_GPIO            GPIO_NUM_16

#define SAMPLE_RATE             16000
#define TONE_FREQ_HZ            440

// 소리 크기. 너무 크면 줄이고, 너무 작으면 키우세요.
// 최대는 대략 30000 근처지만 처음에는 6000 정도가 안전합니다.
#define TONE_AMPLITUDE          6000

#define BUFFER_FRAMES           256

static i2s_chan_handle_t tx_chan = NULL;

static void i2s_speaker_init(void)
{
    printf("Initializing I2S speaker output...\n");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        // MAX98357A는 일반적인 Philips I2S 포맷으로 사용
        // 16bit, stereo로 보내고 L/R 둘 다 같은 값을 출력
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_LRC_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    printf("I2S speaker initialized.\n");
    printf("DIN  = GPIO%d\n", I2S_DOUT_GPIO);
    printf("BCLK = GPIO%d\n", I2S_BCLK_GPIO);
    printf("LRC  = GPIO%d\n", I2S_LRC_GPIO);
}

static void speaker_write_tone_ms(int freq_hz, int duration_ms)
{
    int16_t buffer[BUFFER_FRAMES * 2];   // stereo: L, R
    size_t bytes_written = 0;

    int total_frames = (SAMPLE_RATE * duration_ms) / 1000;
    int remaining_frames = total_frames;

    int half_period_samples = SAMPLE_RATE / (freq_hz * 2);

    if (half_period_samples < 1)
    {
        half_period_samples = 1;
    }

    static int sample_index = 0;

    while (remaining_frames > 0)
    {
        int frames_to_write = BUFFER_FRAMES;

        if (remaining_frames < BUFFER_FRAMES)
        {
            frames_to_write = remaining_frames;
        }

        for (int i = 0; i < frames_to_write; i++)
        {
            int16_t sample;

            if (((sample_index / half_period_samples) % 2) == 0)
            {
                sample = TONE_AMPLITUDE;
            }
            else
            {
                sample = -TONE_AMPLITUDE;
            }

            // stereo 출력: 왼쪽/오른쪽에 같은 값
            buffer[i * 2] = sample;
            buffer[i * 2 + 1] = sample;

            sample_index++;
        }

        ESP_ERROR_CHECK(i2s_channel_write(
            tx_chan,
            buffer,
            frames_to_write * 2 * sizeof(int16_t),
            &bytes_written,
            pdMS_TO_TICKS(1000)
        ));

        remaining_frames -= frames_to_write;
    }
}

static void speaker_write_silence_ms(int duration_ms)
{
    int16_t buffer[BUFFER_FRAMES * 2];
    size_t bytes_written = 0;

    memset(buffer, 0, sizeof(buffer));

    int total_frames = (SAMPLE_RATE * duration_ms) / 1000;
    int remaining_frames = total_frames;

    while (remaining_frames > 0)
    {
        int frames_to_write = BUFFER_FRAMES;

        if (remaining_frames < BUFFER_FRAMES)
        {
            frames_to_write = remaining_frames;
        }

        ESP_ERROR_CHECK(i2s_channel_write(
            tx_chan,
            buffer,
            frames_to_write * 2 * sizeof(int16_t),
            &bytes_written,
            pdMS_TO_TICKS(1000)
        ));

        remaining_frames -= frames_to_write;
    }
}

static void speaker_beep_pattern(void)
{
    printf("Beep 1\n");
    speaker_write_tone_ms(440, 300);
    speaker_write_silence_ms(200);

    printf("Beep 2\n");
    speaker_write_tone_ms(660, 300);
    speaker_write_silence_ms(200);

    printf("Beep 3\n");
    speaker_write_tone_ms(880, 300);
    speaker_write_silence_ms(700);
}

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("04 SPEAKER TEST START\n");
    printf("MAX98357A I2S AMP TEST\n");
    printf("DIN  -> GPIO7\n");
    printf("BCLK -> GPIO15\n");
    printf("LRC  -> GPIO16\n");
    printf("Vin  -> 5V\n");
    printf("GND  -> GND\n");
    printf("====================================\n");

    i2s_speaker_init();

    int count = 0;

    while (1)
    {
        printf("\nSpeaker test cycle %d\n", count);
        speaker_beep_pattern();

        count++;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}