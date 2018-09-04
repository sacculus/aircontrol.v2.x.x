/*
brzo_i2c.h -- A fast i2c master for the esp8266 written in assembly language

Copyright (c) 2016 Pascal Kurtansky (pascal at kurtansky dot ch).
All rights reserved.

This file is part of the library brzo_i2c.

Brzo_i2c is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Brzo_i2c is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/*
 * 2018 Michael Kolomiets (michael dot kolomiets at gmail dot com).
 *
 * Library was adapted for use with ESP8266_RTOS_SDK v2.x.x
 * Small changes for maintain multiple I2C bus instances
 *
 * Original project at https://github.com/pasko-zh/brzo_i2c
 *
 */

/*
 * Notes about use of library.
 *
 * Before use of I2C bus it is need to initialize brzo_i2c_bus instance
 * which assotiate SDA and SCL lines with MCU GPIO lines
 *
 * It is possible to allocate multiple I2C bus instances on different GPIO lines
 * Each of i2c bus instances can be addressed by handle
 * The user must control library usage code to avoid GPIO lines overlap
 *
 * Conversation with device on the i2c bus - transaction, should executed
 * in sequence of calls (states):
 * - start transaction
 * - talk with slave device - one or more subsequent calls for read, write or
 *   wait for ACK
 * - finish transaction
 *
 * The user must initiate transaction with certain bus instance by calling
 * "brzo_i2c_start_transaction" and obtain handle of the device transaction
 *
 * Conversation could contain only one operation - waiting for acknowlege from
 * slave device. For example to detect presence of the slave device on desired
 * address.
 *
 * The user must not alternate operations with different devices on the same bus
 * until current running transaction will ended
 *
 * In anyway user must finish current running transaction with call of
 * "brzo_i2c_end_transaction" to avoid locking bus
 */

#ifndef _BRZO_I2C_h
#define _BRZO_I2C_h

#include <espressif/c_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Setting this variable to 1 DISABLES all interrupts during i2c reads and writes
 * Setting this variable to 0 ENABLES all interrupts during i2c reads and writes
 */
#if !defined(BRZO_I2C_DISABLE_INTERRUPTS)
#define BRZO_I2C_DISABLE_INTERRUPTS		0
#endif

/*
 * Handler of I2C bus instance
 */
typedef void * h_brzo_i2c_bus;


/**
 * @brief      Setup instance of I2C bus
 *             Should be called first before use of any other function
 *
 * @param [in] sda_mux
 *             GPIO MUX for SDA line
 * @param [in] sda_gpio
 *             GPIO pin number used for SDA line
 * @param [in] sda_func
 *             GPIO FUNC for SDA line
 * @param [in] scl_mux
 *             GPIO MUX for SCL line
 * @param [in] scl_gpio
 *             GPIO pin number used for SCL line
 * @param [in] scl_func
 *             GPIO FUNC for SCL line
 * @param [in] clk_stretch
 *             Timeout in micro seconds for an i2c slave clock stretch
 *
 * @return     Handler of I2C bus instance
 *             NULL value indicates error
 */
h_brzo_i2c_bus brzo_i2c_setup(
		uint32_t sda_mux, uint8_t sda_gpio, uint8_t sda_func,
		uint32_t scl_mux, uint8_t scl_gpio, uint8_t scl_func,
		uint32_t clk_stretch);


/**
 * @brief      Release instance of I2C bus and free
 *             allocated resources
 *
 * @param [in] bus
 *             Handler of I2C bus instance
 *
 * @return     void.
 */
void brzo_i2c_free(h_brzo_i2c_bus * bus);


/**
 * @brief      Start transaction with I2C slave device
 *
 * @param [in] bus
 *             Handler of I2C bus instance
 * @param [in] address
 *             7 bit I2C slave address
 * @param [in] frequency
 *             SCL frequency in steps of 100 kHz,
 *             allowed range 100-1000 kHz
 *
 * @return     void.
 *             Result of operations in transaction obtained
 *             at end of the transaction.
 */
void brzo_i2c_start_transaction(
		h_brzo_i2c_bus bus,
		uint8_t address,
		uint16_t frequency);


/**
 * @brief      Finsh current transaction with I2C slave device
 *
 * @param [in] bus
 *             Handler of I2C bus instance
 *
 * @return     "0" when success of transaction or error code.
 */
uint8_t brzo_i2c_end_transaction(h_brzo_i2c_bus bus);


/**
 * @brief      Write data to I2C slave device
 *
 * @param [in] bus
 *             Handler of I2C bus instance
 * @param [in] p_data
 *             Pointer to data buffer to write
 * @param [in] length
 *             Count bytes to write
 * @param [in] repeated
 *             Use repeated start
 *
 * @return     void.
 */
void brzo_i2c_write(
		h_brzo_i2c_bus bus,
		uint8_t *p_data,
		uint32_t length,
		bool repeated);


/**
 * @brief      Read data from I2C slave device
 *
 * @param [in] bus
 *             Handler of I2C bus instance
 * @param [in] p_data
 *             Pointer to data buffer to read
 * @param [in] length
 *             Count bytes to read
 * @param [in] repeated
 *             Use repeated start
 *
 * @return     void.
 */
void brzo_i2c_read(
		h_brzo_i2c_bus bus,
		uint8_t *p_data,
		uint32_t length,
		bool repeated);


/**
 * @brief      Wait acknowlege state from I2C slave device
 *
 * @param [in] bus
 *             Handler of I2C bus instance
 * @param [in] timeout
 *             Time to wait in microsecond
 *
 * @return     void.
 */
void brzo_i2c_ack_polling(h_brzo_i2c_bus bus, uint16_t timeout);

#ifdef __cplusplus
}
#endif

#endif
