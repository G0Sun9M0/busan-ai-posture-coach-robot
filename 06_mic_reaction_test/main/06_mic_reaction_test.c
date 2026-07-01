#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_err.h"

/*
    06_mic_reaction_test - SAFE VERSION

    INMP441 마이크 + MAX98357A 스피커 반응 테스트

    핵심:
    마이크와 스피커를 동시에 I2S로 켜지 않고,
    I2S_NUM_0 하나를 번갈아 사용합니다.

    [INMP441 I2S 마이크]
    WS   -> GPIO4
    SCK  -> GPIO5
    SD   -> GPIO6
    VDD  -> 3.3V
    GND  -> GND
    L/R  -> GND

    [MAX98357A I2S 앰프]
    DIN  -> GPIO7
    BCLK -> GPIO15
    LRC  -> GPIO16
    Vin  -> 5V
    GND  -> GND
*/

#define I2S_PORT                  I2S_NUM_0

// INMP441 마이크 핀
#define I2S_MIC_WS_GPIO           GPIO_NUM_4
#define I2S_MIC_SCK_GPIO          GPIO_NUM_5
#define I2S_MIC_SD_GPIO           GPIO_NUM_6

// MAX98357A 스피커 핀
#define I2S_SPK_DOUT_GPIO         GPIO_NUM_7
#define I2S_SPK_BCLK_GPIO         GPIO_NUM_15
#define I2S_SPK_LRC_GPIO          GPIO_NUM_16

#define MIC_SAMPLE_RATE           16000
#define SPK_SAMPLE_RATE           16000

#define MIC_READ_SAMPLE_COUNT     512
#define SPK_BUFFER_FRAMES         256

// 05_mic_test에서 봤던 값 기준으로 조절
#define MIC_TRIGGER_LEVEL         120

// 소리 출력 후 재감지 방지 시간
#define REACTION_COOLDOWN_MS      2000

#define TRIGGER_HIT_COUNT         3 

// 스피커 소리 크기
#define TONE_AMPLITUDE            7000

static i2s_chan_handle_t mic_rx_chan = NULL;
static i2s_chan_handle_t spk_tx_chan = NULL;

static void mic_i2s_start(void)
{
    if (mic_rx_chan != NULL)
    {
        return;
    }

    printf("Starting microphone I2S...\n");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_PORT,
        I2S_ROLE_MASTER
    );

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &mic_rx_chan));

    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_MONO
    );

    // L/R을 GND에 연결했으므로 LEFT 채널
    slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
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

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(mic_rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(mic_rx_chan));

    printf("Microphone I2S started.\n");
}

static void mic_i2s_stop(void)
{
    if (mic_rx_chan == NULL)
    {
        return;
    }

    printf("Stopping microphone I2S...\n");

    i2s_channel_disable(mic_rx_chan);
    i2s_del_channel(mic_rx_chan);
    mic_rx_chan = NULL;

    vTaskDelay(pdMS_TO_TICKS(100));
}

static void speaker_i2s_start(void)
{
    if (spk_tx_chan != NULL)
    {
        return;
    }

    printf("Starting speaker I2S...\n");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_PORT,
        I2S_ROLE_MASTER
    );

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &spk_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SPK_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SPK_BCLK_GPIO,
            .ws   = I2S_SPK_LRC_GPIO,
            .dout = I2S_SPK_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(spk_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(spk_tx_chan));

    printf("Speaker I2S started.\n");
}

static void speaker_i2s_stop(void)
{
    if (spk_tx_chan == NULL)
    {
        return;
    }

    printf("Stopping speaker I2S...\n");

    i2s_channel_disable(spk_tx_chan);
    i2s_del_channel(spk_tx_chan);
    spk_tx_chan = NULL;

    vTaskDelay(pdMS_TO_TICKS(100));
}

static int calculate_mic_level(int32_t *samples, int sample_count)
{
    int64_t sum = 0;

    /*
        1단계: 평균값을 구합니다.
        마이크에는 소리가 없어도 기본적으로 치우친 값이 있을 수 있습니다.
        이걸 DC offset이라고 생각하면 됩니다.
    */
    for (int i = 0; i < sample_count; i++)
    {
        int32_t sample = samples[i] >> 8;
        sum += sample;
    }

    int32_t mean = (int32_t)(sum / sample_count);

    /*
        2단계: 평균값을 뺀 뒤 변화량만 봅니다.
        이렇게 해야 조용할 때 기본값 때문에 계속 반응하는 문제를 줄일 수 있습니다.
    */
    int64_t sum_abs = 0;
    int32_t max_abs = 0;

    for (int i = 0; i < sample_count; i++)
    {
        int32_t sample = samples[i] >> 8;
        int32_t diff = sample - mean;

        if (diff < 0)
        {
            diff = -diff;
        }

        sum_abs += diff;

        if (diff > max_abs)
        {
            max_abs = diff;
        }
    }

    int avg_abs = (int)(sum_abs / sample_count);

    /*
        표시용 레벨.
        너무 민감하면 1000을 2000으로 키우고,
        너무 둔하면 1000을 500으로 줄입니다.
    */
    int level = avg_abs / 1000;

    return level;
}

static void print_level_bar(int level)
{
    int bar_count = level / 5;

    if (bar_count > 30)
    {
        bar_count = 30;
    }

    printf("MIC LEVEL: %4d | ", level);

    for (int i = 0; i < bar_count; i++)
    {
        printf("#");
    }

    if (level >= MIC_TRIGGER_LEVEL)
    {
        printf("  TRIGGER!");
    }

    printf("\n");
}

static void speaker_write_tone_ms(int freq_hz, int duration_ms)
{
    int16_t buffer[SPK_BUFFER_FRAMES * 2];
    size_t bytes_written = 0;

    int total_frames = (SPK_SAMPLE_RATE * duration_ms) / 1000;
    int remaining_frames = total_frames;

    int half_period_samples = SPK_SAMPLE_RATE / (freq_hz * 2);

    if (half_period_samples < 1)
    {
        half_period_samples = 1;
    }

    int sample_index = 0;

    while (remaining_frames > 0)
    {
        int frames_to_write = SPK_BUFFER_FRAMES;

        if (remaining_frames < SPK_BUFFER_FRAMES)
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

            buffer[i * 2]     = sample;
            buffer[i * 2 + 1] = sample;

            sample_index++;
        }

        ESP_ERROR_CHECK(i2s_channel_write(
            spk_tx_chan,
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
    int16_t buffer[SPK_BUFFER_FRAMES * 2];
    size_t bytes_written = 0;

    memset(buffer, 0, sizeof(buffer));

    int total_frames = (SPK_SAMPLE_RATE * duration_ms) / 1000;
    int remaining_frames = total_frames;

    while (remaining_frames > 0)
    {
        int frames_to_write = SPK_BUFFER_FRAMES;

        if (remaining_frames < SPK_BUFFER_FRAMES)
        {
            frames_to_write = remaining_frames;
        }

        ESP_ERROR_CHECK(i2s_channel_write(
            spk_tx_chan,
            buffer,
            frames_to_write * 2 * sizeof(int16_t),
            &bytes_written,
            pdMS_TO_TICKS(1000)
        ));

        remaining_frames -= frames_to_write;
    }
}

static void play_reaction_sound(void)
{
    printf("\n>>> Sound detected! Reaction beep.\n");

    mic_i2s_stop();

    speaker_i2s_start();

    speaker_write_tone_ms(880, 120);
    speaker_write_silence_ms(80);
    speaker_write_tone_ms(1320, 160);
    speaker_write_silence_ms(200);

    speaker_i2s_stop();

    mic_i2s_start();

    printf(">>> Back to microphone listening.\n\n");
}

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("06 MIC REACTION TEST START - SAFE\n");
    printf("INMP441 mic + MAX98357A speaker\n");
    printf("Use one I2S port alternately.\n");
    printf("====================================\n");

    printf("\n[Microphone]\n");
    printf("WS   -> GPIO4\n");
    printf("SCK  -> GPIO5\n");
    printf("SD   -> GPIO6\n");
    printf("VDD  -> 3.3V\n");
    printf("GND  -> GND\n");
    printf("L/R  -> GND\n");

    printf("\n[Speaker]\n");
    printf("DIN  -> GPIO7\n");
    printf("BCLK -> GPIO15\n");
    printf("LRC  -> GPIO16\n");
    printf("Vin  -> 5V\n");
    printf("GND  -> GND\n");

    printf("\nTrigger level = %d\n", MIC_TRIGGER_LEVEL);
    printf("Clap or speak near the microphone.\n");
    printf("====================================\n\n");

    mic_i2s_start();

    int32_t samples[MIC_READ_SAMPLE_COUNT];
    size_t bytes_read = 0;

    int cooldown_ms = 0;
    int loop_count = 0;
    int trigger_hit = 0;

    while (1)
    {
        esp_err_t ret = i2s_channel_read(
            mic_rx_chan,
            samples,
            sizeof(samples),
            &bytes_read,
            pdMS_TO_TICKS(1000)
        );

        if (ret != ESP_OK)
        {
            printf("I2S mic read failed. err=0x%x\n", ret);
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        int sample_count = bytes_read / sizeof(int32_t);

        if (sample_count <= 0)
        {
            printf("No microphone samples read.\n");
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        int level = calculate_mic_level(samples, sample_count);

        // 너무 많이 출력되지 않도록 5번에 한 번만 표시
        if (loop_count % 5 == 0 || level >= MIC_TRIGGER_LEVEL)
        {
            print_level_bar(level);
        }

        if (cooldown_ms > 0)
        {
            cooldown_ms -= 100;
            trigger_hit = 0;
        }
        else
        {
            if (level >= MIC_TRIGGER_LEVEL)
            {
                trigger_hit++;

                printf("Trigger hit: %d / %d\n", trigger_hit, TRIGGER_HIT_COUNT);

                if (trigger_hit >= TRIGGER_HIT_COUNT)
                {
                    play_reaction_sound();
                    cooldown_ms = REACTION_COOLDOWN_MS;
                    trigger_hit = 0;
                }
            }
            else
            {
                trigger_hit = 0;
            }
        }

        loop_count++;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}