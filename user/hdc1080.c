/*
 * hdc1080.c
 *
 * Functions to manipulate with HDC1080 sensor
 * Used brzo_i2c driver library
 *
 *  Created on: Aug 17 2018
 *      Author: Michael Kolomiets <michael.kolomiets@gmail.com>
 */

#include "brzo_i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hdc1080.h"

/*
 * Structure that describes HDC1080 sensor instance
 */
typedef struct
{
	h_brzo_i2c_bus i2c_bus;
	uint8_t i2c_address;
	uint16_t i2c_frequency;
	uint16_t i2c_ack_timeout;
	uint16_t id_manufacturer;
	uint16_t id_device;
	uint16_t id_serial1, id_serial2, id_serial3;
} hdc1080_device_t;

/*
 * Buffer for two byte I2C operations
 */
typedef union
{
	uint8_t b[2];
	uint16_t raw;
} buff_uint16_t;

h_hdc1080 ICACHE_FLASH_ATTR hdc1080_setup(h_brzo_i2c_bus i2c_bus,
		uint8_t i2c_address, uint16_t i2c_frequency, uint16_t i2c_ack_timeout)
{
	hdc1080_device_t *sensor = NULL;

	if ((sensor = os_malloc(sizeof(hdc1080_device_t))) != NULL)
	{
		sensor->i2c_bus = i2c_bus;
		sensor->i2c_address = i2c_address;
		sensor->i2c_frequency = i2c_frequency;
		sensor->i2c_ack_timeout = i2c_ack_timeout;

		/*
		 * Init sensor ID data - manufacturer, device, serial
		 */
		if (hdc1080_get_reg(sensor, HDC1080_REG_MANUFACTURER,
				&(sensor->id_manufacturer)))
			if (hdc1080_get_reg(sensor, HDC1080_REG_DEVICE,
					&(sensor->id_device)))
				if (hdc1080_get_reg(sensor, HDC1080_REG_SERIAL1,
						&(sensor->id_serial1)))
					if (hdc1080_get_reg(sensor, HDC1080_REG_SERIAL2,
							&(sensor->id_serial2)))
						if (hdc1080_get_reg(sensor, HDC1080_REG_SERIAL3,
								&(sensor->id_serial3)))
							return sensor;
		os_free(sensor);
	}

	return NULL;
}

void ICACHE_FLASH_ATTR hdc1080_free(h_hdc1080 sensor)
{
	if (sensor == NULL)
		return;

	hdc1080_device_t *s = (hdc1080_device_t *) sensor;

	os_free(s);
}

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

bool ICACHE_FLASH_ATTR hdc1080_get_status(h_hdc1080 sensor, uint16_t *p_data)
{
	uint16_t buff;

	if (sensor == NULL)
		return FALSE;

	bool result = hdc1080_get_reg(sensor, HDC1080_REG_STATUS, &buff);
	*p_data = (buff & HDC1080_REG_CONFIG_READ_MASK);
	return result;
}

bool ICACHE_FLASH_ATTR hdc1080_set_config(h_hdc1080 sensor, uint16_t data)
{
	if (sensor == NULL)
		return FALSE;

	bool result = hdc1080_set_reg(sensor,
	HDC1080_REG_CONFIGURATION, (data & HDC1080_REG_CONFIG_WRITE_MASK));
	return result;
}

bool ICACHE_FLASH_ATTR hdc1080_get_temperature(h_hdc1080 sensor, float *p_data)
{
	bool result = false;
	uint8_t i2c_result;
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
	buff.b[0] = HDC1080_REG_TEMPERATURE;
	brzo_i2c_write(s->i2c_bus, buff.b, 1, false);
	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(s->i2c_bus) == 0)
	{
		os_printf(
				"[hdc1080_get_temperature] Send measure command successful\n");
		/*
		 * Loop while hdc1080 data ready
		 */
		uint8_t hdc1080_read_count = 0;
		while (HDC1080_DEF_MEAS_RETRIES > hdc1080_read_count++)
		{
			/*
			 * Sleep until measurement retry delay
			 */
			vTaskDelay(HDC1080_DEF_MEAS_DELAY_MS / portTICK_RATE_MS);
			/*
			 * Start I2C transaction
			 */
			brzo_i2c_start_transaction(s->i2c_bus, s->i2c_address,
					s->i2c_frequency);
			brzo_i2c_ack_polling(s->i2c_bus, s->i2c_ack_timeout);
			/*
			 * Receive data
			 */
			brzo_i2c_read(s->i2c_bus, buff.b, 2, false);
			/*
			 * End transaction
			 */
			if ((i2c_result = brzo_i2c_end_transaction(s->i2c_bus)) == 0)
			{
				*p_data = (buff.raw << 8 & 0xFF00) | (buff.raw >> 8 & 0x00FF);
				result = true;
				os_printf("[hdc1080_get_raw] Sensor raw read "
						"transaction done successful\n");
				break;
			}
			else
			{
				os_printf("[hdc1080_get_raw] Sensor raw read "
						"transaction done with I2C error: %d\n", i2c_result);
				/*
				 * Data not ready yet
				 */
				if (i2c_result == 4)
				{
					os_printf("[hdc1080_get_raw] Sensor data not "
							"ready yet, retrying\n");
					continue;
				}
				break;
			}
		}
	}
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	return result;
}

bool ICACHE_FLASH_ATTR hdc1080_get_humidity(h_hdc1080 sensor, float *pData)
{
	bool result = false;
	return result;
}

bool ICACHE_FLASH_ATTR hdc1080_get_raw(h_hdc1080 sensor, uint32_t *pData)
{
	bool result = false;
	uint8_t ret, i2c_result;
	union
	{
		uint8_t b[4];
		uint32_t raw;
	} buff;

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
	buff.b[0] = HDC1080_REG_TEMPERATURE;
	brzo_i2c_write(s->i2c_bus, buff.b, 1, false);
	/*
	 * End transaction
	 */
	i2c_result = brzo_i2c_end_transaction(s->i2c_bus);
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	if (i2c_result == 0)
	{
		/*
		 * Loop while hdc1080 data ready
		 */
		uint8_t hdc1080_read_count = 0;
		while (HDC1080_DEF_MEAS_RETRIES > hdc1080_read_count++)
		{
			/*
			 * Sleep until measurement retry delay
			 */
			vTaskDelay(HDC1080_DEF_MEAS_DELAY_MS / portTICK_RATE_MS);
			/*
			 * Disable interrupts before transaction
			 */
			taskENTER_CRITICAL();
			/*
			 * Start I2C transaction
			 */
			brzo_i2c_start_transaction(s->i2c_bus, s->i2c_address,
					s->i2c_frequency);
			brzo_i2c_ack_polling(s->i2c_bus, s->i2c_ack_timeout);
			/*
			 * Receive data
			 */
			brzo_i2c_read(s->i2c_bus, buff.b, 4, false);
			/*
			 * End transaction
			 */
			i2c_result = brzo_i2c_end_transaction(s->i2c_bus);
			/*
			 * Restore interrupts after transaction
			 */
			taskEXIT_CRITICAL();

			if (i2c_result == 0)
			{
				*pData = buff.raw;
				result = true;
				break;
			}
			else
			{
				if (i2c_result == 4)
					/*
					 * Data not ready yet
					 */
					continue;
				else
					/*
					 * Other I2C error
					 */
					break;
			}
		}
	}
	return result;
}

uint16_t ICACHE_FLASH_ATTR hdc1080_get_manufacturer(h_hdc1080 sensor)
{
	if (sensor == NULL)
		return 0;

	return ((hdc1080_device_t *) sensor)->id_manufacturer;
}

uint16_t ICACHE_FLASH_ATTR hdc1080_get_device(h_hdc1080 sensor)
{
	if (sensor == NULL)
		return 0;

	return ((hdc1080_device_t *) sensor)->id_device;
}

uint64_t ICACHE_FLASH_ATTR hdc1080_get_serial(h_hdc1080 sensor)
{
	if (sensor == NULL)
		return 0;

	hdc1080_device_t *s = (hdc1080_device_t *) sensor;

	return ((uint64_t) (s->id_serial1) << 24 & 0xffff000000)
			| ((uint32_t) (s->id_serial2) << 8 & 0xffff00)
			| (s->id_serial3 >> 8 & 0xff);
}
