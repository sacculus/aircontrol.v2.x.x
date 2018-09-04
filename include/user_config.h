/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/*
 * ESP
 */
#include <espressif/esp_common.h>

/*
 * Drivers
 */
#include <gpio.h>
#include "hdc1080.h"
#include "dht.h"

/*
 * I2C bus on SDA=GPIO4, SCL=GPIO5
 */
#define SDA_PIN  4
#define SDA_BIT  BIT(SDA_PIN)
#define SDA_MUX  PERIPHS_IO_MUX_GPIO4_U
#define SDA_FUNC FUNC_GPIO4

#define SCL_PIN  5
#define SCL_BIT  BIT(SCL_PIN)
#define SCL_MUX  PERIPHS_IO_MUX_GPIO5_U
#define SCL_FUNC FUNC_GPIO5

#define I2C_DO_SCAN_BUS					1

/*
 * [I2C] Address HDC1080 temperature/humidity sensor
 */
#define I2C_ADDRESS_HDC1080 			0x40

/*
 * vHDC1080Task
 */
#define HDC1080_DEF_INTERVAL_MS			5000

/*
 * [I2C] Address CCS811 air quality sensor
 */
#define I2C_ADDRESS_CCS811 0x23
/*
 * [I2C] Bus speed in kbps for CCS811
 */
#define I2C_SPEED_CCS811 100

#endif

