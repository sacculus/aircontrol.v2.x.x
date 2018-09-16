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
#define CCS811_REG_STATUS           0x00
#define CCS811_REG_MEAS_MODE        0x01
#define CCS811_REG_ALG_RESULT_DATA  0x02
#define CCS811_REG_RAW_DATA         0x03
#define CCS811_REG_ENV_DATA         0x05
#define CCS811_REG_NTC              0x06
#define CCS811_REG_THRESHOLDS       0x10
#define CCS811_REG_BASELINE         0x11

#define CCS811_REG_HW_ID            0x20
#define CCS811_REG_HW_VER           0x21
#define CCS811_REG_FW_BOOT_VER      0x23
#define CCS811_REG_FW_APP_VER       0x24

#define CCS811_REG_ERROR_ID         0xe0

#define CCS811_REG_APP_ERASE        0xf1
#define CCS811_REG_APP_DATA         0xf2
#define CCS811_REG_APP_VERIFY       0xf3
#define CCS811_REG_APP_START        0xf4
#define CCS811_REG_SW_RESET         0xff

// status register bits
#define CCS811_STATUS_ERROR         0x01  // error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY      0x08  // new data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID     0x10  // valid application firmware loaded
#define CCS811_STATUS_FW_MODE       0x80  // firmware is in application mode

// error register bits
#define CCS811_ERR_WRITE_REG_INV    0x01  // invalid register address on write
#define CCS811_ERR_READ_REG_INV     0x02  // invalid register address on read
#define CCS811_ERR_MEASMODE_INV     0x04  // invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE   0x08  // maximum sensor resistance exceeded
#define CCS811_ERR_HEATER_FAULT     0x10  // heater current not in range
#define CCS811_ERR_HEATER_SUPPLY    0x20  // heater voltage not applied correctly

/*
 * Time to boot after power on in ms
 */
#define CCS811_PWR_TIMEOUT_MS       20

/*
 * Time to boot after reset in ms
 */
#define CCS811_RST_TIMEOUT_MS       2

/*
 * Time to application start from boot mode in ms
 */
#define CCS811_APP_TIMEOUT_MS       1

/*
 * Calculate system ticks from time in ms
 * Minimum 1 system tick, otherwise - value is
 * rounded up to system ticks
 */
#define TIME_MS_TO_TICKS(time_ms) time_ms < portTICK_RATE_MS ?                \
                                  1 :                                         \
	                              time_ms / portTICK_RATE_MS +                \
                                          ((time_ms % portTICK_RATE_MS > 0) ? \
					                              1 : 0)                      \

/*
 * Structure that describes sensor instance
 */
typedef struct
{
	h_brzo_i2c_bus i2c_bus;
	uint8_t i2c_address;
	uint16_t i2c_frequency;
	uint16_t i2c_ack_timeout;
	uint8_t hw_id;
	uint8_t hw_ver;
	uint16_t fw_boot_ver;
	uint16_t fw_app_ver;
	uint8_t error;
} ccs811_device_t;

/*
 * Structure of status register data
 */
typedef union {
	uint8_t data;
	struct {
		uint8_t error:1;
		uint8_t :2;
		uint8_t data_ready:1;
		uint8_t app_valid:1;
		uint8_t app_verify:1;
		uint8_t app_erase:1;
		uint8_t fw_mode:1;
	};
} ccs811_reg_status_t;

/*
 * Structure of mode register data
 */
typedef union {
	uint8_t data;
	struct {
		uint8_t :2;
		uint8_t int_threshold:1;
		uint8_t int_dataready:1;
		uint8_t mode:3;
		uint8_t :1;
	};
} ccs811_reg_mode_t;

/*
 * Forward functions declarations
 */
bool ccs811_get_reg(
		ccs811_device_t *sensor,
		uint8_t reg,
		uint8_t length,
		uint8_t * p_data);

bool ccs811_set_reg(
		ccs811_device_t *sensor,
		uint8_t reg,
		uint8_t length,
		uint8_t * p_data);


h_ccs811 ICACHE_FLASH_ATTR ccs811_setup(
		h_brzo_i2c_bus i2c_bus,
		uint8_t i2c_address,
		uint16_t i2c_frequency,
		uint16_t i2c_ack_timeout,
		bool reset_at_start)
{
	ccs811_device_t *sensor = NULL;
    ccs811_reg_status_t status;
    const static uint8_t reset_sequence[4] = { 0x11, 0xe5, 0x72, 0x8a };

	if ((sensor = os_malloc(sizeof(ccs811_device_t))) != NULL)
	{
		sensor->i2c_bus = i2c_bus;
		sensor->i2c_address = i2c_address;
		sensor->i2c_frequency = i2c_frequency;
		sensor->i2c_ack_timeout = i2c_ack_timeout;
		sensor->error = 0;

		/*
		 * Assume that sensor just powered up
		 * so sleep power up delay
		 */
		vTaskDelay(TIME_MS_TO_TICKS(CCS811_PWR_TIMEOUT_MS));

	    /*
	     * Reset device
	     */
		if (reset_at_start)
		{
			/*
			 * Send "magic" reset sequence to sensor
			 */
		    if (ccs811_set_reg(sensor, CCS811_REG_SW_RESET,
		    		4, (uint8_t *) reset_sequence))
		    {
		        /*
		         * Wait until device boot will done after reset
		         * Typical value is up to 2 ms according manual
		         * Minimum 1 system tick, otherwise - value is
		         * rounded up to system ticks
		         */
		        vTaskDelay(TIME_MS_TO_TICKS(CCS811_RST_TIMEOUT_MS));

			}
	    }

		/*
		 * Check that sensor ready to work
		 *
		 * Read the sensor status register
		 */
		if (ccs811_get_reg(sensor, CCS811_REG_STATUS, 1, &(status.data)))
		{
			/*
			 * Check if sensor reported an error
			 */
			if (!status.error)
			{
				/*
				 * Check if sensor is in bootloader mode,
				 * it has to switch to the application mode first
				 */
				if (!(status.fw_mode))
				{
					/*
					 * Check whether valid application firmware is loaded
					 */
					if (status.app_valid)
					{
						/*
						 * Switch to application mode
						 */
						if (ccs811_set_reg(sensor, CCS811_REG_APP_START, 0, 0))
						{
							/*
							 * Wait for starting the sensor app
							 */
							vTaskDelay(TIME_MS_TO_TICKS(CCS811_APP_TIMEOUT_MS));
						}
					}
				}
			}
		}

		/*
		 * Get the status to check whether sensor switched to application mode
		 */
		if (ccs811_get_reg(sensor, CCS811_REG_STATUS, 1, &(status.data)))
		{
			/*
			 * Check if sensor is in application mode
			 */
			if ((status.fw_mode) && (status.app_valid))
			{
				/*
				 * Init sensor ID data - hardware ID and version, firmware,
				 * boot and application version
				 */
				if (ccs811_get_reg(sensor, CCS811_REG_HW_ID,
						1, &(sensor->hw_id)))
					if (ccs811_get_reg(sensor, CCS811_REG_HW_VER,
							1, &(sensor->hw_ver)))
						if (ccs811_get_reg(sensor, CCS811_REG_FW_BOOT_VER,
								2, (uint8_t *) &(sensor->fw_boot_ver)))
							if (ccs811_get_reg(sensor, CCS811_REG_FW_APP_VER,
									2, (uint8_t *) &(sensor->fw_app_ver)))
									return (h_ccs811) sensor;
			}
		}
		os_free(sensor);
	}
	return NULL;
}

void ICACHE_FLASH_ATTR ccs811_free(h_ccs811 sensor)
{
	if (sensor == NULL)
		return;

	ccs811_device_t *s = (ccs811_device_t *) sensor;

	os_free(s);
}

bool ICACHE_FLASH_ATTR ccs811_set_mode(h_ccs811 sensor, ccs811_mode_t mode)
{
	ccs811_reg_mode_t reg;

	/*
	 * Check if sensor handler is valid
	 */
	if (sensor == NULL)
		return FALSE;

	ccs811_device_t *s = (ccs811_device_t *) sensor;

	/*
	 * Get current register value
	 */
	if (ccs811_get_reg(s, CCS811_REG_MEAS_MODE, 1, &(reg.data)))
	{
		/*
		 * Check if new mode is different from current sensor mode
		 */
		if (mode == reg.mode)
		{
			return true;
		}

		/*
		 * Set new mode
		 */
		reg.mode = mode;

		/*
		 * Write new value to sensor register
		 */
		if (ccs811_set_reg(s, CCS811_REG_MEAS_MODE, 1, &(reg.data)))
		{
			return true;
		}
	}

	return false;
}

bool ICACHE_FLASH_ATTR ccs811_get_results(
		h_ccs811 sensor,
		uint16_t *p_tvoc,
		uint16_t *p_eco2,
		uint8_t *p_current,
		uint16_t *p_voltage)
{
	uint8_t buff[8];
	ccs811_reg_status_t status;

	/*
	 * Check if sensor handler is valid
	 */
	if (sensor == NULL)
		return FALSE;

	ccs811_device_t *s = (ccs811_device_t *) sensor;

	if (ccs811_get_reg(s, CCS811_REG_ALG_RESULT_DATA, 8, buff))
	{
		status.data = buff[4];
		if (!(status.error) && (status.data_ready))
		{
			*p_eco2 = buff[0] *256 + buff[1];
			*p_tvoc = buff[2] *256 + buff[3];
			*p_current = (buff[6] >> 2) & 0x3f;
			*p_voltage = (buff[6] & 0x03) * 256 + buff[7];

			return TRUE;
		}
		if (status.error)
		{
			ccs811_get_reg(s, CCS811_REG_ERROR_ID, 1, &(s->error));
		}
		// TODO: Process NO data stateS
	}

	return FALSE;
}

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


bool ICACHE_FLASH_ATTR ccs811_get_reg(
		ccs811_device_t * sensor,
		uint8_t reg,
		uint8_t length,
		uint8_t * p_data)
{
	bool result = false;
	uint8_t buff[8];

	if (sensor == NULL)
		return FALSE;

	ccs811_device_t *s = (ccs811_device_t *) sensor;
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
			*p_data = ((uint16_t) buff[0] << 8 & 0xff00) | buff[1];
		result = true;
	}
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();
	return result;
}

bool ICACHE_FLASH_ATTR ccs811_set_reg(
		ccs811_device_t *sensor,
		uint8_t reg,
		uint8_t length,
		uint8_t * p_data)
{
	bool result = true;

	if (sensor == NULL)
		return FALSE;

	ccs811_device_t *s = (ccs811_device_t *) sensor;

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
	brzo_i2c_write(s->i2c_bus, p_data, 2, false);
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