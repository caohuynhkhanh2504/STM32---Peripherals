/**
 * @file ph4502c_sensor.h
 *
 * @brief STM32 HAL Library for PH4502C Sensor
 *
 * This comprehensive library allows you to interface with the PH4502C sensor
 * for pH and temperature measurements using STM32 HAL. It includes calibration
 * options and flexible configuration.
 * 
 */

#ifndef PH4502C_SENSOR_H
#define PH4502C_SENSOR_H

#include "stm32f1xx_hal.h"

/// Default calibration value for the PH4502C sensor.
#define PH4502C_DEFAULT_CALIBRATION 14.8f

/// Default reading interval (in milliseconds) between pH readings.
#define PH4502C_DEFAULT_READING_INTERVAL 100

/// Default number of pH readings to average.
#define PH4502C_DEFAULT_READING_COUNT 10

/// Default ADC resolution for the PH4502C sensor.
#define PH4502C_DEFAULT_ADC_RESOLUTION 4096.0f

/// Operating voltage for the PH4502C sensor.
#define PH4502C_VOLTAGE 3.3f

/// Voltage that represents a neutral pH reading (pH = 7).
#define PH4502C_MID_VOLTAGE 1.65f

/// Rate of change in voltage per unit change in pH.
#define PH4502C_PH_VOLTAGE_PER_PH 0.18f

/**
 * 
 * @brief PH4502C Sensor structure definition.
 *
 * This structure stores all necessary parameters for interfacing with the PH4502C sensor.
 * 
 */
typedef struct {
    ADC_HandleTypeDef* hadc;        ///< ADC handle for reading sensor values.
    uint32_t ph_level_channel;      ///< ADC channel connected to the pH sensor.
    uint32_t temp_channel;          ///< ADC channel connected to the temperature sensor.
    float calibration;              ///< Calibration value for pH sensor.
    int reading_interval;           ///< Time interval between readings.
    int reading_count;              ///< Number of readings to average.
    float adc_resolution;           ///< ADC resolution (e.g., 4096 for 12-bit ADC).
} PH4502C_Sensor;

/**
 * @brief Initialize the PH4502C sensor structure.
 *
 * @param sensor Pointer to the PH4502C_Sensor structure.
 * @param hadc Pointer to the ADC handle.
 * @param ph_level_channel ADC channel for pH sensor.
 * @param temp_channel ADC channel for temperature sensor.
 * @param calibration Calibration value (default: PH4502C_DEFAULT_CALIBRATION).
 * @param reading_interval Time interval between consecutive pH readings (ms).
 * @param reading_count Number of pH readings to average.
 * @param adc_resolution ADC resolution for the STM32 (default: PH4502C_DEFAULT_ADC_RESOLUTION).
 */
void PH4502C_Init(PH4502C_Sensor* sensor, ADC_HandleTypeDef* hadc, uint32_t ph_level_channel, uint32_t temp_channel,
                  float calibration, int reading_interval, int reading_count, float adc_resolution);

/**
 * @brief Recalibrate the pH sensor.
 *
 * @param sensor Pointer to the PH4502C_Sensor structure.
 * @param calibration New calibration value.
 */
void PH4502C_Recalibrate(PH4502C_Sensor* sensor, float calibration);

/**
 * @brief Read and calculate the pH level.
 *
 * @param sensor Pointer to the PH4502C_Sensor structure.
 * @return Calculated pH level.
 */
float PH4502C_ReadPhLevel(PH4502C_Sensor* sensor);

/**
 * @brief Read and calculate a single pH level (without averaging).
 *
 * @param sensor Pointer to the PH4502C_Sensor structure.
 * @return Single pH reading.
 */
float PH4502C_ReadPhLevelSingle(PH4502C_Sensor* sensor);

/**
 * @brief Read the temperature from the sensor.
 *
 * @param sensor Pointer to the PH4502C_Sensor structure.
 * @return Temperature reading in degrees Celsius.
 */
int PH4502C_ReadTemp(PH4502C_Sensor* sensor);

#endif /* PH4502C_SENSOR_H */
