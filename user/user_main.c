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

#include "user_config.h"

#include <uart.h>
#include <gpio.h>
#include "brzo_i2c.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

h_brzo_i2c_bus volatile i2c_bus_1;

h_hdc1080 volatile hdc1080_1 = NULL;

//volatile bool is_hdc1080_present, is_ccs811_present;

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void hdc1080_task (void *params)
{
	union {
		uint8_t b[4];
		uint16_t w[2];
		uint32_t raw;
	} buff;
	float t, h;
	uint8_t i;
	int dht_t, dht_h;
	char str[32];

    while(1)
    {
        vTaskDelay (HDC1080_DEF_INTERVAL_MS / portTICK_RATE_MS);
        /*
         * Read DHT22
         */
        if(dht_read(&dht_t, &dht_h) == 0)
        {
        	sprintf(str, "t = %.2f\th = %.2f",
        			(float)dht_t / 10,
					(float)dht_h / 10);
        	os_printf("[DHT22] Sensor read:\t%s\n", str);
        }
        if (hdc1080_get_raw(hdc1080_1, &(buff.raw)))
        {
        	/*
        	 * Temperature value
        	 * Care about byte order!!!
        	 */
        	t = (float)((buff.b[0] << 8) | buff.b[1]) * 165 / 65536 - 40;

        	/*
        	 * Humidity value
        	 * Care about byte order!!!
        	 */
        	h = (float)((buff.b[2] << 8) | buff.b[3]) * 100 / 65536;

        	sprintf(str, "t = %.2f\th = %.2f", t, h);
        	os_printf("[HDC1080] Sensor read:\t%s\n", str);
        }
        else
        	os_printf("[HDC1080] Sensor read raw error\n");
    }
}

/*****************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
******************************************************************************/
void user_init(void)
{
	/*
	 * Disable WiFi
	 */
	wifi_set_opmode(NULL_MODE);

    /*
     * Enable debug output to UART1
     */
    UART_ConfigTypeDef uart_config;
    uart_config.baud_rate = BIT_RATE_74880;
    uart_config.data_bits = UART_WordLength_8b;
    uart_config.parity = USART_Parity_None;
    uart_config.stop_bits = USART_StopBits_1;
    uart_config.flow_ctrl = USART_HardwareFlowControl_None;
    uart_config.UART_RxFlowThresh = 120;
    uart_config.UART_InverseMask = UART_None_Inverse;
    UART_ParamConfig(UART0, &uart_config);
    UART_SetPrintPort(UART0);

    printf("=== Firmware SDK version: %s\n", system_get_sdk_version());

    /*
     * [DHT22] Init routine
     */
    dht_init();

    /*
     * [I2C] Init bus instance (SDA = GPIO4, SCL = GPIO5)
     */
    if ((i2c_bus_1 = brzo_i2c_setup(PERIPHS_IO_MUX_GPIO4_U, 4, FUNC_GPIO4, PERIPHS_IO_MUX_GPIO5_U, 5, FUNC_GPIO5, 1000)) != NULL)
    {
//    	is_hdc1080_present = false;
//    	is_ccs811_present = false;

    	/*
    	 * [HDC1080] Init sensor instance (default parameters)
    	 */
    	if ((hdc1080_1 = hdc1080_setup(i2c_bus_1, HDC1080_I2C_DEF_ADDRESS, HDC1080_I2C_DEF_SPEED, HDC1080_I2C_DEF_ACK_TIMEOUT)) != NULL)
    	{
//        	is_hdc1080_present = true;

        	/*
        	 * [HDC1080] Get base sensor properties
        	 */
    		os_printf("[HDC1080] Manufacturer 0x%04x\n",
    				hdc1080_get_manufacturer(hdc1080_1));
    		os_printf("[HDC1080] Device ID 0x%04x\n",
    				hdc1080_get_device(hdc1080_1));
    		os_printf("[HDC1080] Serial 0x%010x\n",
    				hdc1080_get_serial(hdc1080_1));
    	}

    	/*
    	 * [HDC1080] Buffer for operations with registers
    	 */
    	uint16_t buffer;

    	/*
    	 * [HDC1080] Get current status
    	 */
    	if (hdc1080_get_status(hdc1080_1, &buffer))
    		os_printf("[HDC1080] Current status: 0x%04x\n", buffer);

    	/*
    	 * Initialize HDC1080 sensor with configuration:
    	 * -- acquisition mode = temperature+humidity (bit 12 = 1)
    	 * -- 14 bit temperature resolution (bit 10 = 0)
    	 * -- 14 bit humidity resolution (bits 9:8 = 00)
    	 * -- other bits by default as described in documentation
    	 */
    	buffer = HDC1080_REG_CONFIG_WRITE_MASK & HDC1080_CONFIG_MODE_ALL;

    	/*
    	 * [HDC1080] Write new configuration
    	 */
    	if (hdc1080_set_config(hdc1080_1, buffer))
    		os_printf("[HDC1080] Configuration updated, "
    				"waiting 20 mS for sensor start");

    	/*
    	 * [HDC1080] Sleep 20 ms for sensor boot properly
    	 */
    	vTaskDelay(20 / portTICK_RATE_MS);

    	/*
    	 * [HDC1080] Check updated status
    	 */
    	if (hdc1080_get_status(hdc1080_1, &buffer))
    	{
    		os_printf("[HDC1080] New status: 0x%04x\n", buffer);
    		os_printf("[HDC1080] Sensor configured and ready to job");

    	}
//    	else
//    		is_hdc1080_present = false;
    }

    /*
     * Start HDC1080 periodical task
     */
    xTaskCreate(hdc1080_task, (signed char *)"HDC1080_Task", 512, NULL, 2, NULL);
}

