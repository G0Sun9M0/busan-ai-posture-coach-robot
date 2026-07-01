#ifndef KQ_PCA9685_CONSTANTS_H_
#define KQ_PCA9685_CONSTANTS_H_

#include <driver/i2c_types.h>
#include <driver/gpio.h>
#include <stdint.h>

// PCA9685 register map, oscillator/prescale timing, and the fixed angle<->count
// map shared by ALL channels (NOT per-unit calibrated). SG90 per-unit pulse
// tolerance is absorbed mechanically: each leg is mounted to the physical pose
// the 90-degree (SERVO_MID) command produces. There is no trim subsystem and no
// per-channel calibration sweep.

namespace Pca9685Constants
{
    static constexpr uint8_t I2C_ADDRESS = 0x40;

    // Dedicated I2C bus for the PCA9685 (I2C1); the OLED owns I2C0.
    static constexpr i2c_port_t I2C_PORT     = I2C_NUM_1;
    static constexpr gpio_num_t SDA_GPIO     = GPIO_NUM_1;  // freed by removing the battery ADC
    static constexpr gpio_num_t SCL_GPIO     = GPIO_NUM_2;  // GPIO1/2 are not strapping pins (0/3/45/46)
    static constexpr uint32_t   SCL_SPEED_HZ = 400 * 1000;  // sole device on the bus; Fm+ up to ~1MHz possible

    static constexpr uint8_t REG_MODE1      = 0x00;
    static constexpr uint8_t REG_MODE2      = 0x01;
    static constexpr uint8_t REG_LED0_ON_L  = 0x06;  // LEDn base; per channel = REG_LED0_ON_L + REG_STRIDE*channel
    static constexpr uint8_t REG_LED0_OFF_H = 0x09;  // LEDn OFF high byte; bit4 = full-OFF
    static constexpr uint8_t REG_PRESCALE   = 0xFE;

    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_AI      = 0x20;  // auto-increment for the 4-byte setPwm write
    static constexpr uint8_t MODE1_SLEEP   = 0x10;
    static constexpr uint8_t MODE2_OUTDRV  = 0x04;  // totem-pole output (correct for servos)
    static constexpr uint8_t FULL_OFF_BIT  = 0x10;  // bit4 of LEDn_OFF_H = full OFF (limp safe-state)

    static constexpr uint32_t OSC_HZ          = 25000000;  // internal oscillator; tune per scope if period drifts
    static constexpr int      PWM_RESOLUTION  = 4096;      // 12-bit
    static constexpr int      SERVO_FREQ_HZ   = 50;        // 20ms period (SG90)
    static constexpr int      OSC_START_DELAY_US = 500;    // settle window after SLEEP cleared (datasheet)
    static constexpr int      REG_STRIDE      = 4;         // LEDn registers stride by 4
    static constexpr int      TIMEOUT_MS      = 100;       // per-transaction I2C timeout

    // prescale = round(OSC_HZ / (PWM_RESOLUTION * SERVO_FREQ_HZ)) - 1 = 121 (0x79) at 25MHz/4096/50Hz.
    // Computed in Pca9685::setFreq() at runtime rather than hard-coded.

    // Fixed angle<->count map (conservative defaults, all channels). ~4.88us/count at 50Hz/12-bit.
    static constexpr int SERVO_MIN = 102;  //   0 degrees, ~ 500us
    static constexpr int SERVO_MID = 307;  //  90 degrees, ~1500us (mechanical center)
    static constexpr int SERVO_MAX = 491;  // 180 degrees, ~2400us
}

#endif // KQ_PCA9685_CONSTANTS_H_
