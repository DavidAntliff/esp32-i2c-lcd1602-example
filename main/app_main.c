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

// LCD1602
#define LCD_NUM_ROWS 2
#define LCD_NUM_COLUMNS 32
#define LCD_NUM_VISIBLE_COLUMNS 16

// LCD2004
// #define LCD_NUM_ROWS               4
// #define LCD_NUM_COLUMNS            40
// #define LCD_NUM_VISIBLE_COLUMNS    20

// Undefine USE_STDIN if no stdin is available (e.g. no USB UART) - a fixed delay will occur instead of a wait for a keypress.
#define USE_STDIN 1
// #undef USE_STDIN

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0 // disabled
#define I2C_MASTER_RX_BUF_LEN 0 // disabled
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

// uart_rx_one_char_block() causes a watchdog trigger, so use the non-blocking
// uart_rx_one_char() and delay briefly to reset the watchdog.
static uint8_t _wait_for_user(void)
{
    uint8_t c = 0;

#ifdef USE_STDIN
    while (!c)
    {
        STATUS s = uart_rx_one_char(&c);
        if (s == OK)
        {
            printf("%c", c);
        }
        vTaskDelay(1);
    }
#else
    vTaskDelay(1000 / portTICK_RATE_MS);
#endif
    return c;
}

void lcd1602_task(void *pvParameter)
{
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    // Set up the SMBus
    smbus_info_t *smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_PERIOD_MS));

    // Set up the LCD1602 device with backlight off
    i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    // turn off backlight
    ESP_LOGI(TAG, "backlight off");
    _wait_for_user();
    i2c_lcd1602_set_backlight(lcd_info, false);

    // turn on backlight
    ESP_LOGI(TAG, "backlight on");
    _wait_for_user();
    i2c_lcd1602_set_backlight(lcd_info, true);

    ESP_LOGI(TAG, "cursor on");
    _wait_for_user();
    i2c_lcd1602_set_cursor(lcd_info, true);

    ESP_LOGI(TAG, "display A at 0,0");
    _wait_for_user();
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_char(lcd_info, 'A');

    ESP_LOGI(TAG, "display B at 8,0");
    _wait_for_user();
    i2c_lcd1602_move_cursor(lcd_info, 8, 0);
    i2c_lcd1602_write_char(lcd_info, 'B');

    ESP_LOGI(TAG, "display C at 15,1");
    _wait_for_user();
    i2c_lcd1602_move_cursor(lcd_info, 15, 1);
    i2c_lcd1602_write_char(lcd_info, 'C');

    ESP_LOGI(TAG, "move to 0,1 and blink"); // cursor should still be on
    _wait_for_user();
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_set_blink(lcd_info, true);

    ESP_LOGI(TAG, "display DE and move cursor back onto D");
    _wait_for_user();
    i2c_lcd1602_write_char(lcd_info, 'D');
    i2c_lcd1602_set_right_to_left(lcd_info);
    i2c_lcd1602_write_char(lcd_info, 'E');
    i2c_lcd1602_set_left_to_right(lcd_info);

    ESP_LOGI(TAG, "disable display");
    _wait_for_user();
    i2c_lcd1602_set_display(lcd_info, false);

    ESP_LOGI(TAG, "display F at 7,1 (display disabled)");
    _wait_for_user();
    i2c_lcd1602_move_cursor(lcd_info, 7, 1);
    i2c_lcd1602_write_char(lcd_info, 'F');

    ESP_LOGI(TAG, "enable display");
    _wait_for_user();
    i2c_lcd1602_set_display(lcd_info, true);

    ESP_LOGI(TAG, "disable blink");
    _wait_for_user();
    i2c_lcd1602_set_blink(lcd_info, false); // cursor should still be on

    ESP_LOGI(TAG, "disable cursor");
    _wait_for_user();
    i2c_lcd1602_set_cursor(lcd_info, false);

    ESP_LOGI(TAG, "display alphabet from 0,0"); // should overflow to second line at "ABC..."
    _wait_for_user();
    i2c_lcd1602_home(lcd_info);
    i2c_lcd1602_write_string(lcd_info, "abcdefghijklmnopqrstuvwxyz0123456789.,-+ABCDEFGHIJKLMNOPQRSTUVWXYZ");

    ESP_LOGI(TAG, "scroll display left 8 places slowly");
    _wait_for_user();
    for (int i = 0; i < 8; ++i)
    {
        i2c_lcd1602_scroll_display_left(lcd_info);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "scroll display right 8 places quickly");
    _wait_for_user();
    for (int i = 0; i < 8; ++i)
    {
        i2c_lcd1602_scroll_display_right(lcd_info);
    }

    ESP_LOGI(TAG, "move to 8,0 and show cursor");
    _wait_for_user();
    i2c_lcd1602_move_cursor(lcd_info, 8, 0);
    i2c_lcd1602_set_cursor(lcd_info, true);

    ESP_LOGI(TAG, "move cursor 5 places to the right");
    _wait_for_user();
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_move_cursor_right(lcd_info);
    }

    ESP_LOGI(TAG, "move cursor 3 places to the left");
    _wait_for_user();
    for (int i = 0; i < 3; ++i)
    {
        i2c_lcd1602_move_cursor_left(lcd_info);
    }

    ESP_LOGI(TAG, "enable auto-scroll and display >>>>>");
    _wait_for_user();
    i2c_lcd1602_set_auto_scroll(lcd_info, true);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "change address counter to decrement (right to left) and display <<<<<");
    _wait_for_user();
    i2c_lcd1602_set_right_to_left(lcd_info);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '<');
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "disable auto-scroll and display +++++");
    _wait_for_user();
    i2c_lcd1602_set_auto_scroll(lcd_info, false);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '+');
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "set left_to_right and display >>>>>");
    _wait_for_user();
    i2c_lcd1602_set_left_to_right(lcd_info);
    for (int i = 0; i < 5; ++i)
    {
        i2c_lcd1602_write_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "clear display and disable cursor");
    _wait_for_user();
    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_set_cursor(lcd_info, false);

    ESP_LOGI(TAG, "create and display custom characters");
    _wait_for_user();
    // https://github.com/agnunez/ESP8266-I2C-LCD1602/blob/master/examples/CustomChars/CustomChars.ino
    uint8_t bell[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
    uint8_t note[8] = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
    uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
    uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
    uint8_t duck[8] = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
    uint8_t check[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
    uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
    uint8_t retarrow[8] = {0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, bell);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_1, note);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_2, clock);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_3, heart);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_4, duck);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_5, check);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_6, cross);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_7, retarrow);

    // after defining custom characters, DDRAM address must be set by home() or moving the cursor
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_0);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_1);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_2);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_3);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_4);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_5);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_6);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_7);

    ESP_LOGI(TAG, "display special characters");
    _wait_for_user();
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

    ESP_LOGI(TAG, "display all characters (loop)");
    _wait_for_user();
    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_set_cursor(lcd_info, true);
    uint8_t c = 0;
    uint8_t col = 0;
    uint8_t row = 0;
    while (1)
    {
        i2c_lcd1602_write_char(lcd_info, c);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "col %d, row %d, char 0x%02x", col, row, c);
        ++c;
        ++col;
        if (col >= LCD_NUM_VISIBLE_COLUMNS)
        {
            ++row;
            if (row >= LCD_NUM_ROWS)
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
