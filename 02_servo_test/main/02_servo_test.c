#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define I2C_MASTER_NUM              I2C_NUM_1
#define I2C_MASTER_SDA_IO           1
#define I2C_MASTER_SCL_IO           2
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define PCA9685_ADDR                0x40

#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01
#define PCA9685_PRESCALE            0xFE
#define PCA9685_LED0_ON_L           0x06

#define SERVO_MIN_US                600
#define SERVO_MAX_US                2400
#define SERVO_PERIOD_US             20000

#define CH_FL                       0   // Front Left  : 앞 왼쪽
#define CH_FR                       1   // Front Right : 앞 오른쪽
#define CH_RL                       2   // Rear Left   : 뒤 왼쪽
#define CH_RR                       3   // Rear Right  : 뒤 오른쪽

#define CENTER_ANGLE                90
#define TEST_LEFT_ANGLE             60
#define TEST_RIGHT_ANGLE            120
#define TEST_DELAY_MS               800

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    );
}

static esp_err_t pca9685_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        PCA9685_ADDR,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t pca9685_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        PCA9685_ADDR,
        &reg,
        1,
        value,
        1,
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t pca9685_probe(void)
{
    uint8_t mode1 = 0;
    return pca9685_read_reg(PCA9685_MODE1, &mode1);
}

static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_count, uint16_t off_count)
{
    if (channel > 15)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;

    uint8_t data[5] = {
        reg,
        (uint8_t)(on_count & 0xFF),
        (uint8_t)((on_count >> 8) & 0x0F),
        (uint8_t)(off_count & 0xFF),
        (uint8_t)((off_count >> 8) & 0x0F)
    };

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        PCA9685_ADDR,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );
}

static uint16_t servo_angle_to_count(int angle)
{
    if (angle < 0)
    {
        angle = 0;
    }

    if (angle > 180)
    {
        angle = 180;
    }

    int pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
    uint16_t count = (uint16_t)((pulse_us * 4096) / SERVO_PERIOD_US);

    return count;
}

static void servo_write_angle(uint8_t channel, int angle)
{
    uint16_t off_count = servo_angle_to_count(angle);

    esp_err_t ret = pca9685_set_pwm(channel, 0, off_count);

    if (ret == ESP_OK)
    {
        printf("CH%d -> %d degree\n", channel, angle);
    }
    else
    {
        printf("CH%d write failed. angle=%d, err=0x%x\n", channel, angle, ret);
    }
}

static esp_err_t pca9685_init_50hz(void)
{
    printf("Initializing PCA9685 at address 0x%02X\n", PCA9685_ADDR);

    esp_err_t ret;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x00);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    /*
        PCA9685 PWM frequency formula:
        prescale = round(25MHz / (4096 * frequency)) - 1

        For 50Hz servo PWM:
        prescale = round(25000000 / (4096 * 50)) - 1
                 = about 121
    */
    uint8_t prescale = 121;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x10);      // sleep
    if (ret != ESP_OK) return ret;

    ret = pca9685_write_reg(PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) return ret;

    ret = pca9685_write_reg(PCA9685_MODE2, 0x04);      // output driver mode
    if (ret != ESP_OK) return ret;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x20);      // auto increment
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    ret = pca9685_write_reg(PCA9685_MODE1, 0xA0);      // restart + auto increment
    if (ret != ESP_OK) return ret;

    printf("PCA9685 initialized: 50Hz servo PWM\n");

    return ESP_OK;
}

static void servo_all_center(void)
{
    printf("\n");
    printf("Move all servos to center position: 90 degree\n");

    servo_write_angle(CH_FL, CENTER_ANGLE);
    servo_write_angle(CH_FR, CENTER_ANGLE);
    servo_write_angle(CH_RL, CENTER_ANGLE);
    servo_write_angle(CH_RR, CENTER_ANGLE);
}

static void test_one_servo(uint8_t channel, const char *name)
{
    printf("\n");
    printf("------------------------------------\n");
    printf("Testing %s / CH%d\n", name, channel);
    printf("------------------------------------\n");

    servo_write_angle(channel, TEST_LEFT_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));

    servo_write_angle(channel, TEST_RIGHT_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));

    servo_write_angle(channel, CENTER_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));
}

static void test_all_servos_once(void)
{
    test_one_servo(CH_FL, "Front Left");
    test_one_servo(CH_FR, "Front Right");
    test_one_servo(CH_RL, "Rear Left");
    test_one_servo(CH_RR, "Rear Right");
}

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("02 SERVO TEST START\n");
    printf("PCA9685 + SG90 x 4\n");
    printf("SDA = GPIO%d, SCL = GPIO%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    printf("Final position will be 90 degree.\n");
    printf("====================================\n");

    esp_err_t ret = i2c_master_init();

    if (ret != ESP_OK)
    {
        printf("I2C init failed. err=0x%x\n", ret);

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ret = pca9685_probe();

    if (ret != ESP_OK)
    {
        printf("\n");
        printf("PCA9685 not found at address 0x%02X. err=0x%x\n", PCA9685_ADDR, ret);
        printf("Check wiring:\n");
        printf("PCA9685 VCC -> ESP32-S3 3V3\n");
        printf("PCA9685 GND -> ESP32-S3 GND\n");
        printf("PCA9685 SDA -> ESP32-S3 GPIO1\n");
        printf("PCA9685 SCL -> ESP32-S3 GPIO2\n");
        printf("PCA9685 OE  -> GND\n");

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    printf("PCA9685 found at address 0x%02X\n", PCA9685_ADDR);

    ret = pca9685_init_50hz();

    if (ret != ESP_OK)
    {
        printf("PCA9685 init failed. err=0x%x\n", ret);

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // 처음에는 전체 서보를 90도로 정렬
    servo_all_center();
    vTaskDelay(pdMS_TO_TICKS(1500));

    // CH0, CH1, CH2, CH3를 한 번씩 테스트
    test_all_servos_once();

    // 마지막에는 반드시 전체 서보를 90도로 정렬
    printf("\n");
    printf("All servo tests finished.\n");
    printf("Final center alignment: 90 degree\n");

    servo_all_center();
    vTaskDelay(pdMS_TO_TICKS(1500));

    printf("\n");
    printf("Done. All servos are holding 90 degree.\n");
    printf("You can now attach servo horns/legs at this center position.\n");

    // 더 이상 움직이지 않고 대기
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}