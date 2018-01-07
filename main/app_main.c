/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file app_main.c
 * @brief Example application for the LCD1602 16x2 Character Dot Matrix LCD display via I2C backpack..
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"

#include "smbus.h"
#include "i2c-lcd1602.h"

#define TAG "app"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

// WARNING: ESP32 does not support blocking input from stdin yet, so this polls
// the UART and effectively hangs up the SDK.
static uint8_t _getchar(void)
{
    uint8_t c = 0;
    while (!c)
    {
       STATUS s = uart_rx_one_char(&c);
       if (s == OK) {
          printf("%c", c);
       }
    }
    return c;
}

void lcd1602_task(void * pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // Set up the LCD1602 device with backlight off
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    i2c_lcd1602_init(lcd_info, smbus_info, true);

    // turn off backlight
    ESP_LOGI(TAG, "backlight off");
    _getchar();
    i2c_lcd1602_set_backlight(lcd_info, false);

    // turn on backlight
    ESP_LOGI(TAG, "backlight on");
    _getchar();
    i2c_lcd1602_set_backlight(lcd_info, true);

    ESP_LOGI(TAG, "cursor on");
    _getchar();
    i2c_lcd1602_set_cursor(lcd_info, true);

    ESP_LOGI(TAG, "display A at 0,0");
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_char(lcd_info, 'A');

    ESP_LOGI(TAG, "display B at 8,0");
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 8, 0);
    i2c_lcd1602_write_char(lcd_info, 'B');

    ESP_LOGI(TAG, "display C at 15,1");
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 15, 1);
    i2c_lcd1602_write_char(lcd_info, 'C');

    ESP_LOGI(TAG, "move to 0,1 and blink");  // cursor should still be on
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_set_blink(lcd_info, true);

    ESP_LOGI(TAG, "disable display");
    _getchar();
    i2c_lcd1602_set_display(lcd_info, false);

    ESP_LOGI(TAG, "enable display");
    _getchar();
    i2c_lcd1602_set_display(lcd_info, true);

    ESP_LOGI(TAG, "disable blink");
    _getchar();
    i2c_lcd1602_set_blink(lcd_info, false);  // cursor should still be on

    ESP_LOGI(TAG, "disable cursor");
    _getchar();
    i2c_lcd1602_set_cursor(lcd_info, false);

    ESP_LOGI(TAG, "display alphabet at 0,0")  // should overflow to second line at "ABC..."
    _getchar();
    i2c_lcd1602_home(lcd_info);
    i2c_lcd1602_write_string(lcd_info, "abcdefghijklmnopqrstuvwxyz0123456789.,-+ABCDEFGHIJKLMNOPQRSTUVWXYZ");

    ESP_LOGI(TAG, "scroll left 8 places slowly");
    _getchar();
    for (int i = 0; i < 8; ++i)
    {
        i2c_lcd1602_scroll_display_left(lcd_info);
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "scroll right 8 places quickly");
    _getchar();
    for (int i = 0; i < 8; ++i)
    {
        i2c_lcd1602_scroll_display_right(lcd_info);
    }

    ESP_LOGI(TAG, "move to 8,0 and show cursor");
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 8, 0);
    i2c_lcd1602_set_cursor(lcd_info, true);

    ESP_LOGI(TAG, "move cursor 5 places to the right");
    _getchar();
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_move_cursor_right(lcd_info);
    }

    ESP_LOGI(TAG, "move cursor 3 places to the left");
    _getchar();
    for (int i = 0; i < 3; ++i)
    {
        i2c_lcd1602_move_cursor_left(lcd_info);
    }

    ESP_LOGI(TAG, "enable auto-scroll and display >>>>>");
    _getchar();
    i2c_lcd1602_set_auto_scroll(lcd_info, true);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "change address counter to decrement (right to left) and display <<<<<");
    _getchar();
    i2c_lcd1602_set_right_to_left(lcd_info);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '<');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "disable auto-scroll and display +++++");
    _getchar();
    i2c_lcd1602_set_auto_scroll(lcd_info, false);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '+');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "set left_to_right and display >>>>>");
    _getchar();
    i2c_lcd1602_set_left_to_right(lcd_info);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "clear display");
    _getchar();
    i2c_lcd1602_clear(lcd_info);

    ESP_LOGI(TAG, "create custom character and display");
    _getchar();
    // https://github.com/agnunez/ESP8266-I2C-LCD1602/blob/master/examples/CustomChars/CustomChars.ino
    uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
    uint8_t note[8]  = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
    uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
    uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
    uint8_t duck[8]  = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
    uint8_t check[8] = {0x0, 0x1 ,0x3, 0x16, 0x1c, 0x8, 0x0};
    uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
    uint8_t retarrow[8] = { 0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};
    i2c_lcd1602_create_char(lcd_info, 0, bell);
    i2c_lcd1602_create_char(lcd_info, 1, note);
    i2c_lcd1602_create_char(lcd_info, 2, clock);
    i2c_lcd1602_create_char(lcd_info, 3, heart);
    i2c_lcd1602_create_char(lcd_info, 4, duck);
    i2c_lcd1602_create_char(lcd_info, 5, check);
    i2c_lcd1602_create_char(lcd_info, 6, cross);
    i2c_lcd1602_create_char(lcd_info, 7, retarrow);

    // after defining custom characters, DDRAM address must be set by home() or moving the cursor

    ESP_LOGI(TAG, "display custom characters (twice)");
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    for (int i = 0; i < 16; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, i);
    }

    ESP_LOGI(TAG, "display special characters");
    _getchar();
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_ALPHA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_BETA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_THETA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_PI);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_OMEGA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_SIGMA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_INFINITY);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DEGREE);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_ARROW_LEFT);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_ARROW_RIGHT);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_SQUARE);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DOT);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DIVIDE);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_BLOCK);

    ESP_LOGI(TAG, "display all characters");
    _getchar();
    i2c_lcd1602_clear(lcd_info);
    uint8_t c = 0;
    uint8_t col = 0;
    uint8_t row = 0;
    while (1)
    {
        i2c_lcd1602_write_char(lcd_info, c);
        vTaskDelay(120 / portTICK_RATE_MS);
        ESP_LOGI(TAG, "col %d, row %d, char 0x%02x", col, row, c);
        ++c;
        ++col;
        if (col >= I2C_LCD1602_NUM_VISIBLE_COLUMNS)
        {
            ++row;
            if (row >= I2C_LCD1602_NUM_ROWS)
            {
                row = 0;
            }
            col = 0;
            i2c_lcd1602_move_cursor(lcd_info, col, row);
        }
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&lcd1602_task, "lcd1602_task", 4096, NULL, 5, NULL);
}

