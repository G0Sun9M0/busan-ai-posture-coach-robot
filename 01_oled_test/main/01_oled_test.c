#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           41
#define I2C_MASTER_SCL_IO           42
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define OLED_ADDR_DEFAULT           0x3C
#define OLED_WIDTH                  128
#define OLED_HEIGHT                 64
#define OLED_PAGES                  8

static uint8_t s_oled_addr = OLED_ADDR_DEFAULT;

#define FONT_INDEX(c) ((c) - ' ')

static const uint8_t font5x7[96][5] = {
    [FONT_INDEX(' ')] = {0x00, 0x00, 0x00, 0x00, 0x00},
    [FONT_INDEX('-')] = {0x08, 0x08, 0x08, 0x08, 0x08},
    [FONT_INDEX('=')] = {0x14, 0x14, 0x14, 0x14, 0x14},
    [FONT_INDEX(':')] = {0x00, 0x36, 0x36, 0x00, 0x00},
    [FONT_INDEX('.')] = {0x00, 0x60, 0x60, 0x00, 0x00},
    [FONT_INDEX('?')] = {0x02, 0x01, 0x51, 0x09, 0x06},

    [FONT_INDEX('0')] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    [FONT_INDEX('1')] = {0x00, 0x42, 0x7F, 0x40, 0x00},
    [FONT_INDEX('2')] = {0x42, 0x61, 0x51, 0x49, 0x46},
    [FONT_INDEX('3')] = {0x21, 0x41, 0x45, 0x4B, 0x31},
    [FONT_INDEX('4')] = {0x18, 0x14, 0x12, 0x7F, 0x10},
    [FONT_INDEX('5')] = {0x27, 0x45, 0x45, 0x45, 0x39},
    [FONT_INDEX('6')] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
    [FONT_INDEX('7')] = {0x01, 0x71, 0x09, 0x05, 0x03},
    [FONT_INDEX('8')] = {0x36, 0x49, 0x49, 0x49, 0x36},
    [FONT_INDEX('9')] = {0x06, 0x49, 0x49, 0x29, 0x1E},

    [FONT_INDEX('A')] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    [FONT_INDEX('B')] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    [FONT_INDEX('C')] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    [FONT_INDEX('D')] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    [FONT_INDEX('E')] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    [FONT_INDEX('F')] = {0x7F, 0x09, 0x09, 0x09, 0x01},
    [FONT_INDEX('G')] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    [FONT_INDEX('H')] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    [FONT_INDEX('I')] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    [FONT_INDEX('J')] = {0x20, 0x40, 0x41, 0x3F, 0x01},
    [FONT_INDEX('K')] = {0x7F, 0x08, 0x14, 0x22, 0x41},
    [FONT_INDEX('L')] = {0x7F, 0x40, 0x40, 0x40, 0x40},
    [FONT_INDEX('M')] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    [FONT_INDEX('N')] = {0x7F, 0x04, 0x08, 0x10, 0x7F},
    [FONT_INDEX('O')] = {0x3E, 0x41, 0x41, 0x41, 0x3E},
    [FONT_INDEX('P')] = {0x7F, 0x09, 0x09, 0x09, 0x06},
    [FONT_INDEX('Q')] = {0x3E, 0x41, 0x51, 0x21, 0x5E},
    [FONT_INDEX('R')] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    [FONT_INDEX('S')] = {0x46, 0x49, 0x49, 0x49, 0x31},
    [FONT_INDEX('T')] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    [FONT_INDEX('U')] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    [FONT_INDEX('V')] = {0x1F, 0x20, 0x40, 0x20, 0x1F},
    [FONT_INDEX('W')] = {0x3F, 0x40, 0x38, 0x40, 0x3F},
    [FONT_INDEX('X')] = {0x63, 0x14, 0x08, 0x14, 0x63},
    [FONT_INDEX('Y')] = {0x07, 0x08, 0x70, 0x08, 0x07},
    [FONT_INDEX('Z')] = {0x61, 0x51, 0x49, 0x45, 0x43},
};

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

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    return i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        I2C_MASTER_RX_BUF_DISABLE,
        I2C_MASTER_TX_BUF_DISABLE,
        0
    );
}

static esp_err_t i2c_probe(uint8_t address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

static bool i2c_scan_and_select_oled(void)
{
    bool oled_found = false;

    printf("\n");
    printf("====================================\n");
    printf("I2C Scan Start\n");
    printf("SDA = GPIO%d, SCL = GPIO%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    printf("====================================\n");

    for (uint8_t address = 1; address < 127; address++)
    {
        esp_err_t ret = i2c_probe(address);

        if (ret == ESP_OK)
        {
            printf("I2C device found at address 0x%02X\n", address);

            if (address == 0x3C || address == 0x3D)
            {
                s_oled_addr = address;
                oled_found = true;
            }
        }
    }

    if (oled_found)
    {
        printf("OLED selected address: 0x%02X\n", s_oled_addr);
    }
    else
    {
        printf("OLED not found. Check VCC, GND, SDA, SCL wiring.\n");
    }

    return oled_found;
}

static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        s_oled_addr,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t oled_write_data(const uint8_t *data, size_t len)
{
    uint8_t buffer[17];

    while (len > 0)
    {
        size_t chunk = len;
        if (chunk > 16)
        {
            chunk = 16;
        }

        buffer[0] = 0x40;
        memcpy(&buffer[1], data, chunk);

        esp_err_t ret = i2c_master_write_to_device(
            I2C_MASTER_NUM,
            s_oled_addr,
            buffer,
            chunk + 1,
            pdMS_TO_TICKS(100)
        );

        if (ret != ESP_OK)
        {
            return ret;
        }

        data += chunk;
        len -= chunk;
    }

    return ESP_OK;
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_write_cmd(0xB0 + page);
    oled_write_cmd(0x00 + (col & 0x0F));
    oled_write_cmd(0x10 + ((col >> 4) & 0x0F));
}

static void oled_clear(void)
{
    uint8_t zeros[OLED_WIDTH];
    memset(zeros, 0x00, sizeof(zeros));

    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        oled_set_cursor(page, 0);
        oled_write_data(zeros, OLED_WIDTH);
    }
}

static void oled_clear_line(uint8_t page)
{
    uint8_t zeros[OLED_WIDTH];
    memset(zeros, 0x00, sizeof(zeros));

    oled_set_cursor(page, 0);
    oled_write_data(zeros, OLED_WIDTH);
    oled_set_cursor(page, 0);
}

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    oled_write_cmd(0xAE);
    oled_write_cmd(0x20);
    oled_write_cmd(0x00);
    oled_write_cmd(0xB0);
    oled_write_cmd(0xC8);
    oled_write_cmd(0x00);
    oled_write_cmd(0x10);
    oled_write_cmd(0x40);
    oled_write_cmd(0x81);
    oled_write_cmd(0x7F);
    oled_write_cmd(0xA1);
    oled_write_cmd(0xA6);
    oled_write_cmd(0xA8);
    oled_write_cmd(0x3F);
    oled_write_cmd(0xA4);
    oled_write_cmd(0xD3);
    oled_write_cmd(0x00);
    oled_write_cmd(0xD5);
    oled_write_cmd(0x80);
    oled_write_cmd(0xD9);
    oled_write_cmd(0xF1);
    oled_write_cmd(0xDA);
    oled_write_cmd(0x12);
    oled_write_cmd(0xDB);
    oled_write_cmd(0x40);
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);

    oled_clear();
}

static void oled_write_char(char c)
{
    if (c >= 'a' && c <= 'z')
    {
        c = c - 32;
    }

    if (c < ' ' || c > '~')
    {
        c = '?';
    }

    uint8_t char_data[6];

    memcpy(char_data, font5x7[FONT_INDEX(c)], 5);
    char_data[5] = 0x00;

    oled_write_data(char_data, sizeof(char_data));
}

static void oled_print_line(uint8_t page, const char *text)
{
    oled_clear_line(page);

    while (*text)
    {
        oled_write_char(*text);
        text++;
    }
}

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("01 OLED TEST START\n");
    printf("====================================\n");

    ESP_ERROR_CHECK(i2c_master_init());

    bool oled_found = i2c_scan_and_select_oled();

    if (!oled_found)
    {
        printf("\n");
        printf("OLED test stopped.\n");
        printf("Check wiring:\n");
        printf("OLED VCC -> 3V3\n");
        printf("OLED GND -> GND\n");
        printf("OLED SDA -> GPIO41\n");
        printf("OLED SCL -> GPIO42\n");

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    oled_init();

    oled_print_line(0, "ESP32-S3 OLED");
    oled_print_line(1, "OLED TEST OK");
    oled_print_line(2, "SDA=41 SCL=42");

    char addr_line[22];
    snprintf(addr_line, sizeof(addr_line), "ADDR 0X%02X", s_oled_addr);
    oled_print_line(3, addr_line);

    int count = 0;
    char count_line[22];

    while (1)
    {
        snprintf(count_line, sizeof(count_line), "COUNT %04d", count);
        oled_print_line(5, count_line);

        printf("OLED running... count = %d, address = 0x%02X\n", count, s_oled_addr);

        count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}