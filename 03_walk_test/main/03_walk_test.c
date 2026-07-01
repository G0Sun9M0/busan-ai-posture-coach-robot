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

// 채널 배치
#define CH_FL                       0   // 앞 왼쪽
#define CH_FR                       1   // 앞 오른쪽
#define CH_RL                       2   // 뒤 왼쪽
#define CH_RR                       3   // 뒤 오른쪽

// 90도는 현재 바닥에 붙은 자세라고 가정
#define FLAT_ANGLE                  90

/*
    일어서는 자세 기본값

    현재 가정:
    왼쪽 다리는 45도 쪽으로 가면 일어남
    오른쪽 다리는 135도 쪽으로 가면 일어남

    만약 반대로 엎어지면:
    STAND_FL 135
    STAND_FR 45
    STAND_RL 135
    STAND_RR 45
    처럼 바꾸면 됩니다.
*/
#define STAND_FL                    10
#define STAND_FR                    170
#define STAND_RL                    10
#define STAND_RR                    170

// 걷기 동작 폭
#define WALK_SWING                  8

// 부드러운 이동 단계 수
#define SMOOTH_STEPS                20

// 부드러운 이동 간격
#define SMOOTH_DELAY_MS             35

// 걷기 한 자세 유지 시간
#define WALK_HOLD_MS                350

// 걸음 사이 중심 대기 시간
#define STEP_CENTER_MS              120

typedef struct
{
    int fl;
    int fr;
    int rl;
    int rr;
} robot_pose_t;

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
    int output_angle = angle;

    // CH0, 앞 왼쪽 서보만 회전 방향이 반대라서 각도 반전
    if (channel == CH_FL)
    {
        output_angle = 180 - angle;
    }

    uint16_t off_count = servo_angle_to_count(output_angle);

    esp_err_t ret = pca9685_set_pwm(channel, 0, off_count);

    if (ret != ESP_OK)
    {
        printf("Servo CH%d write failed. angle=%d, output=%d, err=0x%x\n",
               channel,
               angle,
               output_angle,
               ret);
    }
}

static esp_err_t pca9685_init_50hz(void)
{
    printf("Initializing PCA9685 50Hz PWM...\n");

    esp_err_t ret;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x00);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t prescale = 121;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x10);
    if (ret != ESP_OK) return ret;

    ret = pca9685_write_reg(PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) return ret;

    ret = pca9685_write_reg(PCA9685_MODE2, 0x04);
    if (ret != ESP_OK) return ret;

    ret = pca9685_write_reg(PCA9685_MODE1, 0x20);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    ret = pca9685_write_reg(PCA9685_MODE1, 0xA0);
    if (ret != ESP_OK) return ret;

    printf("PCA9685 initialized OK\n");

    return ESP_OK;
}

static void set_pose(robot_pose_t pose)
{
    servo_write_angle(CH_FL, pose.fl);
    servo_write_angle(CH_FR, pose.fr);
    servo_write_angle(CH_RL, pose.rl);
    servo_write_angle(CH_RR, pose.rr);
}

static int lerp_int(int start, int end, int step, int total_steps)
{
    return start + ((end - start) * step) / total_steps;
}

static void move_smooth(robot_pose_t from, robot_pose_t to, int steps, int delay_ms)
{
    for (int i = 1; i <= steps; i++)
    {
        robot_pose_t current = {
            .fl = lerp_int(from.fl, to.fl, i, steps),
            .fr = lerp_int(from.fr, to.fr, i, steps),
            .rl = lerp_int(from.rl, to.rl, i, steps),
            .rr = lerp_int(from.rr, to.rr, i, steps),
        };

        set_pose(current);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static robot_pose_t pose_flat(void)
{
    robot_pose_t pose = {
        .fl = FLAT_ANGLE,
        .fr = FLAT_ANGLE,
        .rl = FLAT_ANGLE,
        .rr = FLAT_ANGLE
    };

    return pose;
}

static robot_pose_t pose_stand(void)
{
    robot_pose_t pose = {
        .fl = STAND_FL,
        .fr = STAND_FR,
        .rl = STAND_RL,
        .rr = STAND_RR
    };

    return pose;
}

static robot_pose_t pose_walk_a(void)
{
    /*
        대각선 A:
        FL + RR 한 쌍
        FR + RL 반대 쌍

        standing pose를 기준으로 작은 폭만 흔듭니다.
    */
    robot_pose_t pose = {
        .fl = STAND_FL + WALK_SWING,
        .fr = STAND_FR + WALK_SWING,
        .rl = STAND_RL - WALK_SWING,
        .rr = STAND_RR - WALK_SWING
    };

    return pose;
}

static robot_pose_t pose_walk_b(void)
{
    /*
        대각선 B:
        A와 반대 방향
    */
    robot_pose_t pose = {
        .fl = STAND_FL - WALK_SWING,
        .fr = STAND_FR - WALK_SWING,
        .rl = STAND_RL + WALK_SWING,
        .rr = STAND_RR + WALK_SWING
    };

    return pose;
}

static void stand_up_sequence(void)
{
    robot_pose_t flat = pose_flat();
    robot_pose_t stand = pose_stand();

    printf("\n");
    printf("Step 0: flat pose, all servos 90 degree\n");
    set_pose(flat);
    vTaskDelay(pdMS_TO_TICKS(800));

    printf("Standing up...\n");
    printf("FL=%d, FR=%d, RL=%d, RR=%d\n", stand.fl, stand.fr, stand.rl, stand.rr);

    move_smooth(flat, stand, SMOOTH_STEPS, SMOOTH_DELAY_MS);

    printf("Standing pose hold 1 second.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
}

static void walk_four_steps(void)
{
    robot_pose_t stand = pose_stand();
    robot_pose_t walk_a = pose_walk_a();
    robot_pose_t walk_b = pose_walk_b();

    for (int step = 1; step <= 4; step++)
    {
        printf("\n");
        printf("====================================\n");
        printf("WALK STEP %d\n", step);
        printf("====================================\n");

        if (step % 2 == 1)
        {
            printf("Pose A\n");
            printf("FL=%d, FR=%d, RL=%d, RR=%d\n",
                   walk_a.fl, walk_a.fr, walk_a.rl, walk_a.rr);

            move_smooth(stand, walk_a, 8, 30);
            vTaskDelay(pdMS_TO_TICKS(WALK_HOLD_MS));
            move_smooth(walk_a, stand, 8, 30);
        }
        else
        {
            printf("Pose B\n");
            printf("FL=%d, FR=%d, RL=%d, RR=%d\n",
                   walk_b.fl, walk_b.fr, walk_b.rl, walk_b.rr);

            move_smooth(stand, walk_b, 8, 30);
            vTaskDelay(pdMS_TO_TICKS(WALK_HOLD_MS));
            move_smooth(walk_b, stand, 8, 30);
        }

        vTaskDelay(pdMS_TO_TICKS(STEP_CENTER_MS));
    }
}

static void sit_down_sequence(void)
{
    robot_pose_t stand = pose_stand();
    robot_pose_t flat = pose_flat();

    printf("\n");
    printf("Sit down sequence start.\n");
    printf("Move from standing pose to 90 degree flat pose.\n");

    move_smooth(stand, flat, SMOOTH_STEPS, SMOOTH_DELAY_MS);

    printf("Sit down done. All servos are now 90 degree.\n");
}

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("03 WALK TEST START\n");
    printf("Stand up first, then walk 4 steps\n");
    printf("PCA9685 + SG90 x 4\n");
    printf("SDA = GPIO%d, SCL = GPIO%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    printf("PCA9685 address = 0x%02X\n", PCA9685_ADDR);
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
        printf("PCA9685 not found. err=0x%x\n", ret);
        printf("Check VCC, GND, SDA, SCL, OE wiring.\n");

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

    stand_up_sequence();

    walk_four_steps();

    printf("\n");
    printf("Walking done. Sit down now.\n");

    sit_down_sequence();

    printf("\n");
    printf("Done. Robot is sitting down at 90 degree.\n");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}