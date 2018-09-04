/*
 * hdc1080.h
 *
 * Functions to manipulate HDC1080 sensor
 * Use brzo_i2c driver library to work with I2C bus
 *
 *  Created on: Aug 17 2018
 *      Author: Michael Kolomiets <michael.kolomiets@gmail.com>
 */

#ifndef INCLUDE_HDC1080_H_
#define INCLUDE_HDC1080_H_

#include <espressif/c_types.h>
#include "brzo_i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Sensor instance handler
 */
typedef void * h_hdc1080;

/*
 * [I2C] Default address for HDC1080
 */
#define HDC1080_I2C_DEF_ADDRESS				0x40

/*
 * [I2C] Bus speed in kbps for HDC1080
 */
#define HDC1080_I2C_DEF_SPEED 				100

/*
 * [I2C] Default ACS pooling timeout in uS for HDC1080
 */
#define HDC1080_I2C_DEF_ACK_TIMEOUT			500

/*
 * [I2C] Default measurement delay in mS for HDC1080
 */
#define HDC1080_DEF_MEAS_DELAY_MS			10
#define HDC1080_DEF_MEAS_RETRIES			15

/*
 * HDC1080 registers
 */
#define HDC1080_REG_TEMPERATURE				0x00
#define HDC1080_REG_HUMIDITY				0x01
#define HDC1080_REG_CONFIGURATION			0x02
#define HDC1080_REG_STATUS					HDC1080_REG_CONFIGURATION
#define HDC1080_REG_SERIAL1					0xfb
#define HDC1080_REG_SERIAL2					0xfc
#define HDC1080_REG_SERIAL3					0xfd
#define HDC1080_REG_MANUFACTURER			0xfe
#define HDC1080_REG_DEVICE					0xff

/*
 * Configuration register values
 */
#define HDC1080_CONFIG_RST					0b1000000000000000
#define HDC1080_CONFIG_HEAT					0b0010000000000000
#define HDC1080_CONFIG_MODE_ONE				0b0000000000000000
#define HDC1080_CONFIG_MODE_ALL				0b0001000000000000
#define HDC1080_CONFIG_TRES_11BIT			0b0000010000000000
#define HDC1080_CONFIG_HRES_11BIT			0b0000000100000000
#define HDC1080_CONFIG_HRES_8BIT			0b0000001000000000

/*
 * Configuration register masks
 */
#define HDC1080_CONFIG_MASK_RST				0b1000000000000000
#define HDC1080_CONFIG_MASK_HEAT			0b0010000000000000
#define HDC1080_CONFIG_MASK_MODE			0b0001000000000000
#define HDC1080_CONFIG_MASK_BTST			0b0000100000000000
#define HDC1080_CONFIG_MASK_TRES			0b0000010000000000
#define HDC1080_CONFIG_MASK_HRES			0b0000001100000000

/*
 * Configuration register write mask
 */
#define HDC1080_REG_CONFIG_WRITE_MASK		0b1011011100000000
/*
 * Configuration register read mask
 */
#define HDC1080_REG_CONFIG_READ_MASK		0b1011111100000000

/**
 * @brief      Setup instance of HDC1080 sensor I2C slave device
 *
 * @param [in] i2c_bus
 *             I2C bus instance handler
 * @param [in] i2c_address
 *             I2C address of device, 7 bit
 * @param [in] i2c_frequency
 *             I2C clock frequency in kHz
 * @param [in] i2c_ack_timeout
 *             I2C slave ACK wait timeout in usec
 *
 * @return     hdc1080_t, NULL:indicates failure,
 *             valid pointer value indicates success.
 */
h_hdc1080 hdc1080_setup(h_brzo_i2c_bus i2c_bus, uint8_t i2c_address,
		uint16_t i2c_frequency, uint16_t i2c_ack_timeout);

/**
 * @brief      Release instance of HDC1080 sensor I2C slave device
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 *
 * @return     void.
 */
void hdc1080_free(h_hdc1080 sensor);

/*
 * @brief      Read value from register
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] reg
 *             Register address
 * @param [in] p_data
 *             Pointer to read data buffer
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_get_reg(h_hdc1080 sensor, uint8_t reg, uint16_t *p_pata);

/*
 * @brief      Write value to register
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] reg
 *             Register address
 * @param [in] data
 *             Data to write
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_set_reg(h_hdc1080 sensor, uint8_t reg, uint16_t data);

/*
 * @brief      Read value from status register
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] p_data
 *             Pointer to read data buffer
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_get_status(h_hdc1080 sensor, uint16_t *p_pata);

/*
 * @brief      Write value to configuration register
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] data
 *             Data to write
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_set_config(h_hdc1080 sensor, uint16_t data);

/*
 * @brief      Read temperature value
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] p_data
 *             Pointer to read data buffer
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_get_temperature(h_hdc1080 sensor, float *p_pata);

/*
 * @brief      Read humidity value
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] p_data
 *             Pointer to read data buffer
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_get_humidity(h_hdc1080 sensor, float *p_pata);

/*
 * @brief      Read raw temperature and humidity value
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 * @param [in] p_data
 *             Pointer to read data buffer
 *             Value should be converted according to sensor manual
 *             First 16 bits contain temperature value
 *             Next 16 bits contain humidity value
 *             Note to reverse octets order in 16 bit words
 *
 * @return     TRUE if read success, FALSE if read fails.
 */
bool hdc1080_get_raw(h_hdc1080 sensor, uint32_t *p_pata);

/*
 * @brief      Get sensor manufacturer ID
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 *
 * @return     Manufacturer ID (always 0x5449).
 */
uint16_t hdc1080_get_manufacturer(h_hdc1080 sensor);

/*
 * @brief      Get sensor device ID
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 *
 * @return     Device ID (always 0x1050 = Texas instruments).
 */
uint16_t hdc1080_get_device(h_hdc1080 sensor);

/*
 * @brief      Get sensor serial number
 *
 * @param [in] sensor
 *             Handler of HDC1080 sensor I2C slave device instance
 *
 * @return     Unique serial number of sensor.
 */
uint64_t hdc1080_get_serial(h_hdc1080 sensor);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_HDC1080_H_ */
