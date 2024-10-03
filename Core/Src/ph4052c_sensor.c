/**
 * @file ph4502c_sensor.c
 *
 * @brief STM32 HAL Library for PH4502C Sensor Implementation
 * 
 */

#include "ph4502c_sensor.h"

/**
 * @brief Initialize the PH4502C sensor structure.
 */
void PH4502C_Init(PH4502C_Sensor* sensor, ADC_HandleTypeDef* hadc, uint32_t ph_level_channel, uint32_t temp_channel,
                  float calibration, int reading_interval, int reading_count, float adc_resolution) {
    sensor->hadc = hadc;
    sensor->ph_level_channel = ph_level_channel;
    sensor->temp_channel = temp_channel;
    sensor->calibration = calibration;
    sensor->reading_interval = reading_interval;
    sensor->reading_count = reading_count;
    sensor->adc_resolution = adc_resolution;
}

/**
 * @brief Recalibrate the pH sensor.
 */
void PH4502C_Recalibrate(PH4502C_Sensor* sensor, float calibration) {
    sensor->calibration = calibration;
}

/**
 * @brief Read and calculate the pH level.
 */
float PH4502C_ReadPhLevel(PH4502C_Sensor* sensor) {
    float total_voltage = 0;
    for (int i = 0; i < sensor->reading_count; i++) {
        total_voltage += PH4502C_ReadPhLevelSingle(sensor);
        HAL_Delay(sensor->reading_interval);
    }
    float avg_voltage = total_voltage / sensor->reading_count;
    return (7.0f - ((avg_voltage - PH4502C_MID_VOLTAGE) / PH4502C_PH_VOLTAGE_PER_PH));
}

/**
 * @brief Read and calculate a single pH level (without averaging).
 */
float PH4502C_ReadPhLevelSingle(PH4502C_Sensor* sensor) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = sensor->ph_level_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(sensor->hadc, &sConfig);
    
    HAL_ADC_Start(sensor->hadc);
    HAL_ADC_PollForConversion(sensor->hadc, HAL_MAX_DELAY);
    uint32_t raw_value = HAL_ADC_GetValue(sensor->hadc);
    HAL_ADC_Stop(sensor->hadc);
    
    float voltage = (raw_value * PH4502C_VOLTAGE) / sensor->adc_resolution;
    return voltage;
}

/**
 * @brief Read the temperature from the sensor.
 */
int PH4502C_ReadTemp(PH4502C_Sensor* sensor) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = sensor->temp_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(sensor->hadc, &sConfig);
    
    HAL_ADC_Start(sensor->hadc);
    HAL_ADC_PollForConversion(sensor->hadc, HAL_MAX_DELAY);
    uint32_t raw_value = HAL_ADC_GetValue(sensor->hadc);
    HAL_ADC_Stop(sensor->hadc);
    
    float voltage = (raw_value * PH4502C_VOLTAGE) / sensor->adc_resolution;
    
    // Convert voltage to temperature (assumed linear relationship, adjust as needed)
    return (int)((voltage - 0.5f) * 100.0f); // Example calculation, modify for your temperature sensor
}
