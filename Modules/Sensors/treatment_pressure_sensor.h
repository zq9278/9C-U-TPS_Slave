#ifndef MODULES_SENSORS_TREATMENT_PRESSURE_SENSOR_H
#define MODULES_SENSORS_TREATMENT_PRESSURE_SENSOR_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT 2U
#define TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW 3U

typedef struct
{
    float sample_buf[TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT][TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW];
    uint8_t sample_count[TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT];
    uint8_t sample_index[TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT];
    TickType_t next_sample_tick;
} TreatmentPressureSensor;

void TreatmentPressureSensor_Init(TreatmentPressureSensor *sensor);
void TreatmentPressureSensor_Poll(TreatmentPressureSensor *sensor,
                                  TickType_t now,
                                  volatile sensor_data_t *out);

#ifdef __cplusplus
}
#endif

#endif
