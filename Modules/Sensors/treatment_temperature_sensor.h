#ifndef MODULES_SENSORS_TREATMENT_TEMPERATURE_SENSOR_H
#define MODULES_SENSORS_TREATMENT_TEMPERATURE_SENSOR_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "system_app.h"
#include "UserDrivers/Ads1248/ads1248.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    UserAds1248Channel next_channel;
    temp_acquire_mode_t mode;
    uint8_t suppress_rtd_fail_log;
} TreatmentTemperatureSensor;

void TreatmentTemperatureSensor_Init(TreatmentTemperatureSensor *sensor,
                                     SemaphoreHandle_t rtd_drdy_sem);
void TreatmentTemperatureSensor_SetMode(TreatmentTemperatureSensor *sensor,
                                        temp_acquire_mode_t mode,
                                        uint8_t suppress_rtd_fail_log);
void TreatmentTemperatureSensor_Poll(TreatmentTemperatureSensor *sensor,
                                     volatile sensor_data_t *out);

#ifdef __cplusplus
}
#endif

#endif
