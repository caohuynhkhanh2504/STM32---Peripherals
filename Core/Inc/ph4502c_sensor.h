#ifndef PH4502C_SENSOR_H
#define PH4502C_SENSOR_H

#include "stm32f1xx_hal.h"

// Define default constants
#define PH4502C_DEFAULT_CALIBRATION    7.0f
#define PH4502C_VOLTAGE                5.0f
#define PH4502C_MID_VOLTAGE            2.5f
#define PH4502C_PH_VOLTAGE_PER_PH      0.18f
#define PH4502C_DEFAULT_READING_COUNT  10
#define PH4502C_DEFAULT_READING_INTERVAL 1000  // Microseconds
#define PH4502C_DEFAULT_ADC_RESOLUTION 4096    // 12-bit ADC resolution

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t ph_level_channel;
    uint32_t temp_channel;
    float calibration;
    int reading_count;
    int reading_interval;
    int adc_resolution;
} PH4502C_Sensor;

void PH4502C_Sensor_Init(PH4502C_Sensor *sensor, ADC_HandleTypeDef *hadc, uint32_t ph_level_channel, uint32_t temp_channel);
void PH4502C_Sensor_Recalibrate(PH4502C_Sensor *sensor, float calibration);
float PH4502C_Sensor_ReadPHLevel(PH4502C_Sensor *sensor);
float PH4502C_Sensor_ReadPHLevelSingle(PH4502C_Sensor *sensor, uint16_t *adc_value);
int PH4502C_Sensor_ReadTemp(PH4502C_Sensor *sensor);

#endif // PH4502C_SENSOR_H
