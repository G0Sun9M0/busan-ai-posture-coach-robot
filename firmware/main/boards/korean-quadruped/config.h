#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// Board hardware configuration for the korean-quadruped board, following the
// xiaozhi per-board config.h convention (compile-time #define). Servo / PCA9685
// / gait domain constants live in their own constexpr headers
// (servo_constants.h, pca9685_constants.h, gait_constants.h).
//
// No app button is declared: the English WakeNet wake word is the sole runtime
// entry point. GPIO0 remains only the dev-board flash/download strap, not a
// firmware input. No battery ADC: GPIO1 is reassigned to the PCA9685 I2C bus.

#include <driver/gpio.h>
#include <driver/i2c_types.h>

// Audio sample rates. The server hello may override the output rate at runtime.
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// INMP441 I2S microphone (RX). L/R tied to GND selects the LEFT slot.
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6

// MAX98357A I2S speaker (TX).
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

// On-board status LED.
#define BUILTIN_LED_GPIO        GPIO_NUM_48

// SSD1306 OLED on its own dedicated I2C bus (I2C0). The PCA9685 servo driver
// lives on a separate I2C1 bus (pca9685_constants.h) so a full display refresh
// and servo writes can never contend on the same controller.
#define DISPLAY_I2C_PORT        I2C_NUM_0
#define DISPLAY_I2C_SDA_PIN     GPIO_NUM_41
#define DISPLAY_I2C_SCL_PIN     GPIO_NUM_42
#define DISPLAY_I2C_ADDR        0x3C
#define DISPLAY_I2C_SPEED_HZ    (400 * 1000)
#define I2C_GLITCH_IGNORE_CNT   7
#define DISPLAY_WIDTH           128
#define DISPLAY_HEIGHT          64
#define DISPLAY_MIRROR_X        true
#define DISPLAY_MIRROR_Y        true

#endif // _BOARD_CONFIG_H_
