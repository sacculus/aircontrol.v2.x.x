/*
 * ccs811.c
 *
 * Functions to manipulate CCS811 sensor
 *
 *  Created on: Sep 05 2018
 *      Author: Michael Kolomiets <michael.kolomiets@gmail.com>
 *
 * Original source from https://github.com/gschorcht/ccs811-esp-idf
 * Original author annotation and legal notice can be read in README.ccs811
 *
 * Original source code was slightly reworked for:
 * - use brzo_i2c as I2C bus library;
 * - some cosmetic changes of coding style;
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ccs811.h"

/* CCS811 register addresses */
#define CCS811_REG_STATUS          0x00
#define CCS811_REG_MEAS_MODE       0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_RAW_DATA        0x03
#define CCS811_REG_ENV_DATA        0x05
#define CCS811_REG_NTC             0x06
#define CCS811_REG_THRESHOLDS      0x10
#define CCS811_REG_BASELINE        0x11

#define CCS811_REG_HW_ID           0x20
#define CCS811_REG_HW_VER          0x21
#define CCS811_REG_FW_BOOT_VER     0x23
#define CCS811_REG_FW_APP_VER      0x24

#define CCS811_REG_ERROR_ID        0xe0

#define CCS811_REG_APP_ERASE       0xf1
#define CCS811_REG_APP_DATA        0xf2
#define CCS811_REG_APP_VERIFY      0xf3
#define CCS811_REG_APP_START       0xf4
#define CCS811_REG_SW_RESET        0xff

// status register bits
#define CCS811_STATUS_ERROR        0x01  // error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY     0x08  // new data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID    0x10  // valid application firmware loaded
#define CCS811_STATUS_FW_MODE      0x80  // firmware is in application mode

// error register bits
#define CCS811_ERR_WRITE_REG_INV   0x01  // invalid register address on write
#define CCS811_ERR_READ_REG_INV    0x02  // invalid register address on read
#define CCS811_ERR_MEASMODE_INV    0x04  // invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE  0x08  // maximum sensor resistance exceeded
#define CCS811_ERR_HEATER_FAULT    0x10  // heater current not in range
#define CCS811_ERR_HEATER_SUPPLY   0x20  // heater voltage not applied correctly

/*
 * Time to boot after reset in usec
 */
#define CCS811_RST_TIMEOUT         2000

/*
 * Time to boot after power on in usec
 */
#define CCS811_PWR_TIMEOUT         20000

/*
 * Structure that describes sensor instance
 */
typedef struct
{
	h_brzo_i2c_bus i2c_bus;
	uint8_t i2c_address;
	uint16_t i2c_frequency;
	uint16_t i2c_ack_timeout;
	uint8_t device_id;
	uint8_t hw_version;
	uint16_t boot_version;
	uint16_t app_version;
} ccs811_device_t;

/*
 * Forward functions declarations
 */
bool ccs811_get_reg(
		ccs811_device_t *sensor,
		uint8_t reg,
		uint8_t length,
		uint8_t *p_data);

bool ccs811_set_reg(
		ccs811_device_t *sensor,
		uint8_t reg,
		uint8_t length,
		uint8_t *p_data);

h_ccs811 ICACHE_FLASH_ATTR ccs811_setup(
		h_brzo_i2c_bus i2c_bus,
		uint8_t i2c_address,
		uint16_t i2c_frequency,
		uint16_t i2c_ack_timeout)
{
	ccs811_device_t *sensor = NULL;
    uint8_t status;
    const static uint8_t reset_sequence[4] = { 0x11, 0xe5, 0x72, 0x8a };

	if ((sensor = os_malloc(sizeof(ccs811_device_t))) != NULL)
	{
		sensor->i2c_bus = i2c_bus;
		sensor->i2c_address = i2c_address;
		sensor->i2c_frequency = i2c_frequency;
		sensor->i2c_ack_timeout = i2c_ack_timeout;

	    /*
	     * Reset device
	     */
	    if (ccs811_set_reg(sensor, CCS811_REG_SW_RESET,
	    		4, (uint8_t *)reset_sequence))
	    {
	        /*
	         * Wait until device boot will done after reset
	         */
	        vTaskDelay((CCS811_RST_TIMEOUT / portTICK_RATE_MS) == 0 ? 1 :
					CCS811_RST_TIMEOUT / portTICK_RATE_MS + 1);
	        if (ccs811_get_reg(sensor, CCS811_REG_STATUS, 1, &status))
	            // if sensor is in bootloader mode (FW_MODE == 0), it has to switch
	            // to the application mode first
	            if (!(status & CCS811_STATUS_FW_MODE))
	            {
	                // check whether valid application firmware is loaded
	                if (!(status & CCS811_STATUS_APP_VALID))
	                {
	                }
	                // switch to application mode
	                if (!ccs811_set_reg(sensor, CCS811_REG_APP_START, 0, 0))
	                {
	                }

	                // wait 100 ms after starting the app
	                vTaskDelay(100/portTICK_RATE_MS);

	                // get the status to check whether sensor switched to application mode
	                if (!ccs811_get_reg(sensor, CCS811_REG_STATUS, 1, &status) ||
	                    !(status & CCS811_STATUS_FW_MODE))
	                {
	                }
	            }

	    }
//		/*
//		 * Init sensor ID data - manufacturer, device, serial
//		 */
//		if (hdc1080_get_reg(sensor, HDC1080_REG_MANUFACTURER,
//				&(sensor->id_manufacturer)))
//			if (hdc1080_get_reg(sensor, HDC1080_REG_DEVICE,
//					&(sensor->id_device)))
//				if (hdc1080_get_reg(sensor, HDC1080_REG_SERIAL1,
//						&(sensor->id_serial1)))
//					if (hdc1080_get_reg(sensor, HDC1080_REG_SERIAL2,
//							&(sensor->id_serial2)))
//						if (hdc1080_get_reg(sensor, HDC1080_REG_SERIAL3,
//								&(sensor->id_serial3)))
//							return sensor;
//		os_free(sensor);
	}

	return NULL;
}

void ICACHE_FLASH_ATTR ccs811_free(h_ccs811 sensor);

bool ICACHE_FLASH_ATTR ccs811_set_mode(h_ccs811 sensor, ccs811_mode_t mode);

bool ICACHE_FLASH_ATTR ccs811_get_results(
		h_ccs811 sensor,
		uint16_t *p_tvoc,
		uint16_t *p_eco2,
		uint8_t *p_current,
		uint16_t *p_voltage);

uint32_t ICACHE_FLASH_ATTR ccs811_get_ntc_resistance(
		h_ccs811 sensor,
		uint32_t resistance);

bool ICACHE_FLASH_ATTR ccs811_set_environmental_data(
		h_ccs811 sensor,
		float temperature,
		float humidity);

bool ICACHE_FLASH_ATTR ccs811_set_interrupt(h_ccs811 sensor, bool enabled);

bool ccs811_set_interrupt_thresholds(
		h_ccs811 sensor,
		uint16_t low,
		uint16_t high,
		uint8_t hysteresis);

uint16_t ICACHE_FLASH_ATTR ccs811_get_baseline(h_ccs811 sensor);

bool ICACHE_FLASH_ATTR ccs811_set_baseline(h_ccs811 sensor, uint16_t baseline);


bool ICACHE_FLASH_ATTR hdc1080_get_reg(h_hdc1080 sensor, uint8_t reg,
		uint16_t *pData)
{
	bool result = false;
	uint8_t buff[2];

	if (sensor == NULL)
		return FALSE;

	hdc1080_device_t *s = (hdc1080_device_t *) sensor;
	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();
	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(s->i2c_bus, s->i2c_address, s->i2c_frequency);
	brzo_i2c_ack_polling(s->i2c_bus, s->i2c_ack_timeout);
	/*
	 * Send register address
	 */
	brzo_i2c_write(s->i2c_bus, &reg, 1, false);
	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(s->i2c_bus) == 0)
	{
		/*
		 * Start I2C transaction
		 */
		brzo_i2c_start_transaction(s->i2c_bus, s->i2c_address,
				s->i2c_frequency);
		brzo_i2c_ack_polling(s->i2c_bus, s->i2c_ack_timeout);
		/*
		 * Receive register data first byte (MSB)
		 */
		brzo_i2c_read(s->i2c_bus, buff, 2, false);
		/*
		 * End transaction
		 */
		if (brzo_i2c_end_transaction(s->i2c_bus) == 0)
			*pData = ((uint16_t) buff[0] << 8 & 0xff00) | buff[1];
		result = true;
	}
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();
	return result;
}

bool ICACHE_FLASH_ATTR hdc1080_set_reg(h_hdc1080 sensor, uint8_t reg,
		uint16_t data)
{
	bool result = true;
	buff_uint16_t buff;

	if (sensor == NULL)
		return FALSE;

	hdc1080_device_t *s = (hdc1080_device_t *) sensor;
	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();
	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(s->i2c_bus, s->i2c_address, s->i2c_frequency);
	brzo_i2c_ack_polling(s->i2c_bus, s->i2c_ack_timeout);
	/*
	 * Send register address
	 */
	brzo_i2c_write(s->i2c_bus, &reg, 1, true);
	/*
	 * Send register data
	 */
	buff.raw = data;
	/*
	 * Send register data
	 */
	brzo_i2c_write(s->i2c_bus, buff.b, 2, false);
	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(s->i2c_bus) != 0)
		result = false;
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();
	return result;
}
