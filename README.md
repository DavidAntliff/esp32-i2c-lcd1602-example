# esp32-i2c-lcd1602-example

[![Platform: ESP-IDF](https://img.shields.io/badge/ESP--IDF-v3.0%2B-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/)
[![Build Status](https://travis-ci.org/DavidAntliff/esp32-i2c-lcd1602-example.svg?branch=master)](https://travis-ci.org/DavidAntliff/esp32-i2c-lcd1602-example)
[![license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

## Introduction

This is an example application for the HD4470-compatible LCD1602 device connected via an I2C backpack.

This application is also usable with an LCD2004 module (20 columns, 4 rows), by uncommenting the `LCD_NUM_ROWS`, `LCD_NUM_COLUMNS`, and `LCD_NUM_VISIBLE_COLUMNS` definitions at the top of `app_main.c`.

It is written and tested for v3.3 of the [ESP-IDF](https://github.com/espressif/esp-idf) environment, using the xtensa-esp32-elf toolchain (gcc version 5.2.0).

Ensure that submodules are cloned:

    $ git clone --recursive https://github.com/DavidAntliff/esp32-i2c-lcd1602-example.git

Build the application with:

    $ cd esp32-i2c-lcd1602-example.git
    $ idf.py menuconfig    # set your serial configuration and the I2C GPIO - see below
    $ idf.py build
    $ idf.py -p (PORT) flash monitor

The program should detect your connected device and display some demonstration text on the LCD.

## Dependencies

This application makes use of the following components (included as submodules):

 * components/[esp32-smbus](https://github.com/DavidAntliff/esp32-smbus)
 * components/[esp32-i2c-lcd1602](https://github.com/DavidAntliff/esp32-i2c-lcd1602)

## Hardware

To run this example, connect one LCD1602 device to two GPIOs on the ESP32 (I2C SDA and SCL). If external pull-up resistors are not provided with the sensor, add a 10 KOhm resistor from each GPIO to the 3.3V supply.

`idf.py menuconfig` can be used to set the I2C GPIOs and LCD1602 device I2C address.

Note that the 3.3V supply may be insufficient to run the display satisfactorily. In this case I suggest using a 5V supply to the LCD display, and using appropriate level shifter circuitry on the I2C SCL and SDA connections.

## Features

This example steps through the features of the esp32-i2c-lcd1602 component. It demonstrates:

 * Display initialisation, disabling and enabling, clearing.
 * Backlight control.
 * Underline and blinking cursor control, including arbitrary cursor movement and homing.
 * Display scrolling.
 * Custom character definition.
 * Display of all characters.

Each step waits for a keypress on stdin before executing. If stdin is not available (no USB/UART available), modify the following code:

```
//#define USE_STDIN  1
#undef USE_STDIN
```

Each step will then wait for one second before proceeding automatically.

## Source Code

The source is available from [GitHub](https://www.github.com/DavidAntliff/esp32-i2c-lcd1602-example).

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

 * [HD44780 Dot Matrix Liquid Crystal Display Controller/Driver datasheet](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf)
 * [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)

## Acknowledgements

 * "I2C" is a registered trademark of Phillips Corporation.
 * "SMBus" is a trademark of Intel Corporation.
 
