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

typedef enum
{
    TEMP_SCAN_STATE_IDLE = 0,
    TEMP_SCAN_STATE_SWITCH_CHANNEL,
    TEMP_SCAN_STATE_WAIT_DISCARD_SAMPLE,
    TEMP_SCAN_STATE_WAIT_VALID_SAMPLE
} treatment_temp_scan_state_t;

/* 温度采样状态机。双通道模式下按“切换 -> 丢首样本 -> 取有效样本”循环推进。 */
typedef struct
{
    UserAds1248Channel next_channel;
    UserAds1248Channel active_channel;
    temp_acquire_mode_t mode;
    uint8_t suppress_rtd_fail_log;
    uint8_t scan_state;
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
