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

/* 温度采样模块运行态。 */
typedef struct
{
    UserAds1248Channel next_channel;
    temp_acquire_mode_t mode;
    uint8_t suppress_rtd_fail_log;
} TreatmentTemperatureSensor;

/* 初始化 ADS1248 温度采样模块。 */
void TreatmentTemperatureSensor_Init(TreatmentTemperatureSensor *sensor,
                                     SemaphoreHandle_t rtd_drdy_sem);
/* 切换温度采样模式。 */
void TreatmentTemperatureSensor_SetMode(TreatmentTemperatureSensor *sensor,
                                        temp_acquire_mode_t mode,
                                        uint8_t suppress_rtd_fail_log);
/* 执行一次温度采样轮询。 */
void TreatmentTemperatureSensor_Poll(TreatmentTemperatureSensor *sensor,
                                     volatile sensor_data_t *out);

#ifdef __cplusplus
}
#endif

#endif
