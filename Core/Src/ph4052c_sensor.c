#include "ph4502c_sensor.h"

void PH4502C_Sensor_Init(PH4502C_Sensor *sensor, ADC_HandleTypeDef *hadc, uint32_t ph_level_channel, uint32_t temp_channel) {
    sensor->hadc = hadc;
    sensor->ph_level_channel = ph_level_channel;
    sensor->temp_channel = temp_channel;
    sensor->calibration = PH4502C_DEFAULT_CALIBRATION;
    sensor->reading_count = PH4502C_DEFAULT_READING_COUNT;
    sensor->reading_interval = PH4502C_DEFAULT_READING_INTERVAL;
    sensor->adc_resolution = PH4502C_DEFAULT_ADC_RESOLUTION;
}

void PH4502C_Sensor_Recalibrate(PH4502C_Sensor *sensor, float calibration) {
    sensor->calibration = calibration;
}

float PH4502C_Sensor_ReadPHLevel(PH4502C_Sensor *sensor) {
    float reading = 0.0f;
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Configure ADC channel for pH sensor
    sConfig.Channel = sensor->ph_level_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(sensor->hadc, &sConfig);

    for (int i = 0; i < sensor->reading_count; i++) {
        HAL_ADC_Start(sensor->hadc);
        HAL_ADC_PollForConversion(sensor->hadc, HAL_MAX_DELAY);
        reading += HAL_ADC_GetValue(sensor->hadc);
        HAL_ADC_Stop(sensor->hadc);

        HAL_Delay(sensor->reading_interval / 1000);  // Convert to milliseconds
    }

    reading = (PH4502C_VOLTAGE / sensor->adc_resolution) * reading / sensor->reading_count;
    reading = sensor->calibration + ((PH4502C_MID_VOLTAGE - reading)) / PH4502C_PH_VOLTAGE_PER_PH;

    return reading;
}

float PH4502C_Sensor_ReadPHLevelSingle(PH4502C_Sensor *sensor, uint16_t *adc_value) {
 
		float reading = *adc_value;
	
    reading = (PH4502C_VOLTAGE / (2*sensor->adc_resolution)) * reading;
    return sensor->calibration + ((PH4502C_MID_VOLTAGE - reading)) / PH4502C_PH_VOLTAGE_PER_PH;
		
}

int PH4502C_Sensor_ReadTemp(PH4502C_Sensor *sensor) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure ADC channel for temperature sensor
    sConfig.Channel = sensor->temp_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(sensor->hadc, &sConfig);

    HAL_ADC_Start(sensor->hadc);
    HAL_ADC_PollForConversion(sensor->hadc, HAL_MAX_DELAY);
    int temp = HAL_ADC_GetValue(sensor->hadc);
    HAL_ADC_Stop(sensor->hadc);

    return temp;
}
