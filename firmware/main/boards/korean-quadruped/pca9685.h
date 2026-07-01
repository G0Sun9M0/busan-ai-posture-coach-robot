#ifndef KQ_PCA9685_H_
#define KQ_PCA9685_H_

#include "pca9685_constants.h"

#include <driver/i2c_master.h>
#include <esp_rom_sys.h>
#include <stdint.h>

// PCA9685 16-channel PWM driver on its own dedicated I2C bus.
//
// Mirrors boards/common/i2c_device.cc (own dev handle + i2c_master_transmit),
// but adds a 5-byte auto-increment setPwm write and, unlike i2c_device.cc, returns
// status (bool) from the runtime write path instead of aborting via ESP_ERROR_CHECK
// -- so GaitController can drop the robot to a limp safe-state on bus failure rather
// than panicking. Single-owner: only the gait task drives it after construction.
class Pca9685
{
private:
    i2c_master_dev_handle_t mDevice;  // IDF opaque handle owned on the dedicated PCA9685 bus

public:
    Pca9685(i2c_master_bus_handle_t i2cBus)
    {
        i2c_device_config_t tConfig = {};
        tConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        tConfig.device_address = Pca9685Constants::I2C_ADDRESS;
        tConfig.scl_speed_hz = Pca9685Constants::SCL_SPEED_HZ;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBus, &tConfig, &mDevice));
    }

    // MODE2 totem-pole output, then 50Hz prescale. Caller must follow with a center
    // setPwm on every channel before relying on outputs (defined first pulse).
    bool begin()
    {
        if (!writeReg(Pca9685Constants::REG_MODE2, Pca9685Constants::MODE2_OUTDRV))
        {
            return false;
        }
        return setFreq(Pca9685Constants::SERVO_FREQ_HZ);
    }

    // PRESCALE may only be written while MODE1.SLEEP is set (datasheet); Adafruit sequence.
    bool setFreq(int freqHz)
    {
        const int tPrescale = computePrescale(freqHz);
        uint8_t tOldMode = 0;
        if (!readReg(Pca9685Constants::REG_MODE1, tOldMode))
        {
            return false;
        }
        const uint8_t tSleep = (uint8_t)((tOldMode & ~Pca9685Constants::MODE1_RESTART) | Pca9685Constants::MODE1_SLEEP);
        if (!writeReg(Pca9685Constants::REG_MODE1, tSleep))
        {
            return false;
        }
        if (!writeReg(Pca9685Constants::REG_PRESCALE, (uint8_t)tPrescale))
        {
            return false;
        }
        if (!writeReg(Pca9685Constants::REG_MODE1, tOldMode))
        {
            return false;
        }
        esp_rom_delay_us(Pca9685Constants::OSC_START_DELAY_US);
        const uint8_t tRestart = (uint8_t)((tOldMode & ~Pca9685Constants::MODE1_SLEEP)
                                           | Pca9685Constants::MODE1_RESTART
                                           | Pca9685Constants::MODE1_AI);
        return writeReg(Pca9685Constants::REG_MODE1, tRestart);
    }

    // Single 5-byte transmit [reg, ON_L=0, ON_H=0, OFF_L, OFF_H] at LED0_ON_L + 4*channel
    // (MODE1.AI auto-increments the 4 LED registers). offCount is a 12-bit OFF tick.
    bool setPwm(int channel, int offCount)
    {
        const uint8_t tReg = (uint8_t)(Pca9685Constants::REG_LED0_ON_L
                                       + Pca9685Constants::REG_STRIDE * channel);
        uint8_t tBuffer[5] = {
            tReg,
            0,
            0,
            (uint8_t)(offCount & 0xFF),
            (uint8_t)((offCount >> 8) & 0x0F),
        };
        return i2c_master_transmit(mDevice, tBuffer, sizeof(tBuffer), Pca9685Constants::TIMEOUT_MS) == ESP_OK;
    }

    // Cut the pulse train on one channel (LEDn_OFF_H full-OFF bit); the servo goes
    // limp. Used by GaitController::enterSafeState() so a fault leaves the robot limp
    // rather than wedged holding a bad pose.
    bool fullOff(int channel)
    {
        const uint8_t tOffHReg = (uint8_t)(Pca9685Constants::REG_LED0_OFF_H
                                           + Pca9685Constants::REG_STRIDE * channel);
        return writeReg(tOffHReg, Pca9685Constants::FULL_OFF_BIT);
    }

    // Raw single-register write. Public because enterSafeState()'s full-OFF path uses it.
    bool writeReg(uint8_t reg, uint8_t value)
    {
        uint8_t tBuffer[2] = { reg, value };
        return i2c_master_transmit(mDevice, tBuffer, sizeof(tBuffer), Pca9685Constants::TIMEOUT_MS) == ESP_OK;
    }

private:
    // out-param byte alongside the bool status (caller-owned), per the I2C read pattern.
    bool readReg(uint8_t reg, uint8_t &outValue)
    {
        return i2c_master_transmit_receive(mDevice, &reg, 1, &outValue, 1, Pca9685Constants::TIMEOUT_MS) == ESP_OK;
    }

    static int computePrescale(int freqHz)
    {
        const double tRaw = (double)Pca9685Constants::OSC_HZ
                            / (Pca9685Constants::PWM_RESOLUTION * (double)freqHz);
        return (int)(tRaw + 0.5) - 1;
    }
};

#endif // KQ_PCA9685_H_
