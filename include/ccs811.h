/*
 * ccs811.h
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

#include <espressif/c_types.h>
#include "brzo_i2c.h"

#ifndef INCLUDE_CCS811_H_
#define INCLUDE_CCS811_H_

/*
 * [I2C] Default address for CCS811
 */
#define CCS811_I2C_DEF_ADDRESS				0x5a

/*
 * [I2C] Bus speed in kbps for CCS811
 */
#define CCS811_I2C_DEF_SPEED 				100

/*
 * [I2C] Default ACS pooling timeout in usec for CCS811
 */
#define CCS811_I2C_DEF_ACK_TIMEOUT			500

/*
 * [I2C] CCS811 clock stretching counter
 */
#define CCS811_I2C_CLOCK_STRETCH  			200

/**
 * [CCS811] Operation modes
 */
typedef enum
{
	ccs811_mode_idle = 0, // Idle, low current mode
	ccs811_mode_1s = 1, // Constant Power mode, IAQ values every 1 s
	ccs811_mode_10s = 2, // Pulse Heating mode, IAQ values every 10 s
	ccs811_mode_60s = 3, // Low Power Pulse Heating, IAQ values every 60 s
	ccs811_mode_250ms = 4  // Constant Power mode, RAW data every 250 ms
} ccs811_mode_t;

/*
 * [CCS811] Sensor instance handler
 */
typedef void * h_ccs811;

/**
 * @brief	    Setup instance of CCS811 sensor I2C slave device
 *
 * The function initializes the CCS811 sensor and checks its availability
 *
 * @param [in]  i2c_bus
 *              I2C bus instance handler
 * @param [in]  i2c_address
 *              I2C address of device, 7 bit
 * @param [in]  i2c_frequency
 *              I2C clock frequency in kHz
 * @param [in]  i2c_ack_timeout
 *              I2C slave ACK wait timeout in usec
 *
 * @return      h_ccs811, NULL value indicates failure,
 *              any valid pointer value indicates success.
 */
h_ccs811 ccs811_setup(
		h_brzo_i2c_bus i2c_bus,
		uint8_t i2c_address,
		uint16_t i2c_frequency,
		uint16_t i2c_ack_timeout,
		bool reset_at_start);

/**
 * @brief       Release instance of CCS811 sensor I2C slave device
 *
 * @param [in]  sensor
 *              Handler of CCS811 sensor I2C slave device instance
 *
 * @return      void.
 */
void ccs811_free(h_ccs811 sensor);

/*
 * @brief       Get sensor internal error code
 *
 * @param [in]  sensor
 *              Handler of CCS811 sensor I2C slave device instance
 *
 * @return      void.
 *
 * TODO: Complete "ccs811_get_error()" function
 */
uint8_t ccs811_get_error_code(h_ccs811 *sensor);

/**
 * @brief 	    Set the operation mode of the sensor
 *
 * The function sets the operating mode of the sensor. If the parameter
 * *mode* is either *ccs811_mode_1s*, *ccs811_mode_10s*, *ccs811_mode_60s*
 * or *ccs811_mode_250ms*, the sensor starts a periodic measurement with
 * the specified period. Function *ccs811_get_results* can then be used at
 * the same rate to get the results.
 *
 * In *ccs811_mode_1s*, *ccs811_mode_10s* and *ccs811_mode_60s*, raw sensor
 * data as well as IAQ values calculated by the  sensor values are available.
 * In *ccs811_mode_250ms*, only raw data are available.
 *
 * In case, parameter mode is *ccs811_mode_idle*, the sensor does not perform
 * any measurements.
 *
 * Please note: Mode timings are subject to typical 2% tolerance due
 * to accuracy of internal sensor clock.
 *
 * Please note: After setting the sensor mode, the sensor needs up to
 * 20 minutes, before accurate readings are generated.
 *
 * Please note: When a sensor operating mode is changed to a new mode with
 * a lower sample rate, e.g., from *ccs811_mode_60s* to *ccs811_mode_1s, it
 * should be placed in *mode_idle* for at least 10 minutes before enabling
 * the new mode.
 *
 * @param [in]  sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param [in]  mode
 *              CCS811 operation mode to set
 *
 * @return      TRUE on success, FALSE on error.
 */
bool ccs811_set_mode(h_ccs811 sensor, ccs811_mode_t mode);

/**
 * @brief	    Get latest IAQ sensor values and/or RAW sensor data
 *
 * The function reads the IAQ sensor values (TVOC and eCO2) and/or the raw
 * sensor data. If some of the results are not needed, the corresponding
 * pointer parameters can be set to NULL.
 *
 * Please note: If the function is called and no new data are available,
 * e.g., due to the sensor mode time tolerance of 2%, the function still
 * returns successfully. In this case, the results of the last measurement
 * are returned and the error code CCS811_DRV_NO_NEW_DATA is set.
 *
 * Please note: In *ccs811_mode_250ms*, only RAW data are available. In
 * that case, the function fails with error_code CCS811_DRV_NO_IAQ_DATA
 * if parameters *iaq_tvoc* and *iaq_eco2* are not NULL.
 *
 * @param [in]  sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param [out] p_tvoc
 *              Pointer to buffer for read value of TVOC total volatile
 *              organic compound (0 - 1187 ppb)
 * @param [out] p_eco2
 *              Pointer to buffer for read value of eCO2 equivalent CO2
 *              (400 - 8192 ppm)
 * @param [out] p_current
 *              Pointer to buffer for read value of current through the
 *              sensor used for measuring (0 - 63 uA)
 * @param [out] p_voltage
 *              Pointer to buffer for read value of voltage across the
 *              sensor measured (0 - 1023 = 1.65 V)
 *
 * @return      TRUE on success, FALSE on error.
 */
bool ccs811_get_results(h_ccs811 sensor, uint16_t *p_tvoc, uint16_t *p_eco2,
		uint8_t *p_current, uint16_t *p_voltage);

/**
 * brief        Get the current resistance of connected NTC thermistor
 *
 * CCS811 supports an external interface for connecting a negative thermal
 * coefficient thermistor (R_NTC) to provide a cost effective and power
 * efficient means of calculating the local ambient temperature. The sensor
 * measures the voltage V_NTC across the R_NTC as well as the voltage V_REF
 * across a connected reference resistor (R_REF).
 *
 * The function returns the current resistance of R_NTC using the equation
 *
 *          R_NTC = R_REF / V_REF * V_NTC
 *
 * Using the data sheet of the NTC, the ambient temperature can be calculated.
 *
 * @param  [in] sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param  [in] resistance
 *              Resistance value of reference resistor (R_REF) in Ohm
 *
 * @return      Current resistance of negative thermal coefficient thermistor
 *              (R_NTC) in Ohm, or 0 on error
 */
uint32_t ccs811_get_ntc_resistance(h_ccs811 sensor, uint32_t resistance);

/*
 * @brief       Set the environmental data for reference
 *
 * If information about the environment are available from another sensor,
 * they can be used by CCS811 to compensate gas readings due to
 * temperature and humidity changes.
 *
 * @param  [in] sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param  [in] temperature
 *              measured temperature in degree Celsius
 * @param  [in] humidity
 *              measured relative humidity in percent
 *
 * @return      TRUE on success, FALSE on error.
 */
bool ccs811_set_environmental_data(h_ccs811 sensor, float temperature,
		float humidity);

/**
 * @brief       Enable or disable data ready interrupt signal *nINT*
 *
 * At the end of each measurement cycle (250ms, 1s, 10s, 60s), CCS811 can
 * optionally trigger an interrupt. The signal *nINT* is driven low as soon
 * as new sensor values are ready to read. It will stop being driven low
 * when sensor data are read with function *ccs811_get_results*.
 *
 * The interrupt is disabled by default.
 *
 * @param  [in] sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param  [in] enabled
 *              If TRUE, the interrupt is enabled, or disabled otherwise
 *
 * @return      TRUE on success, FALSE on error
 */
bool ccs811_set_interrupt(h_ccs811 sensor, bool enabled);

/*
 * @brief       Set eCO2 threshold mode for data ready interrupts
 *
 * The user task can choose that the data ready interrupt is not generated
 * every time when new sensor values become ready but only if the eCO2 value
 * moves from the current range (LOW, MEDIUM, or HIGH) into another range by
 * more than a hysteresis value. Hysteresis is used to prevent multiple
 * interrupts close to a threshold.
 *
 *   LOW     below parameter value *low*
 *   MEDIUM  between parameter values *low* and *high*
 *   HIGH    above parameter value *high* is range HIGH.
 *
 * If all parameters have valid values, the function sets the thresholds and
 * enables the data ready interrupt. Using 0 for all parameters disables the
 * interrupt.
 *
 * The interrupt is disabled by default.
 *
 * @param  [in] sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param  [in] low
 *              Threshold LOW to MEDIUM  (>  400, default 1500)
 * @param  [in] high
 *              Threshold MEDIUM to HIGH (< 8192, default 2500)
 * @param  [in] hysteresis
 *              Hysteresis value (default 50)
 *
 * @return      TRUE on success, FALSE on error
 */
bool ccs811_set_interrupt_thresholds(
		h_ccs811 sensor,
		uint16_t low,
		uint16_t high,
		uint8_t hysteresis);

/*
 * @brief       Get the current baseline value from sensor
 *
 * The sensor supports automatic baseline correction over a minimum time of
 * 24 hours. Using this function, the current baseline value can be saved
 * before the sensor is powered down. This baseline can then be restored after
 * sensor is powered up again to continue the automatic baseline process.
 *
 * @param  [in] sensor
 *              Handler of CCS811 sensor I2C slave device instance
 *
 * @return      Current baseline value on success, or 0 on error
 */
uint16_t ccs811_get_baseline(h_ccs811 sensor);

/*
 * @brief       Write a previously stored baseline value to the sensor
 *
 * The sensor supports automatic baseline correction over a minimum time of
 * 24 hours. Using this function, a previously saved baseline value be
 * restored after the sensor is powered up to continue the automatic baseline
 * process.
 *
 * Please note: The baseline must be written after the conditioning period
 * of 20 min after power up.
 *
 * @param  [in] sensor
 *              Handler of CCS811 sensor I2C slave device instance
 * @param  [in] basline
 *              Value of baseline to be set
 * @return      TRUE on success, FALSE on error
 */
bool ccs811_set_baseline(h_ccs811 sensor, uint16_t baseline);

#endif /* INCLUDE_CCS811_H_ */
