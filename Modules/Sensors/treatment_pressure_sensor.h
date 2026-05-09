#ifndef MODULES_SENSORS_TREATMENT_PRESSURE_SENSOR_H
#define MODULES_SENSORS_TREATMENT_PRESSURE_SENSOR_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 压力传感器当前固定为左右两路。 */
#define TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT 2U
/* 每路压力中值滤波窗口长度。 */
#define TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW 3U

/* 压力采样模块运行态。 */
typedef struct
{
    float sample_buf[TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT][TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW];
    uint8_t sample_count[TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT];
    uint8_t sample_index[TREATMENT_PRESSURE_SENSOR_CHANNEL_COUNT];
    TickType_t next_sample_tick;
} TreatmentPressureSensor;

/* 初始化压力采样模块。 */
void TreatmentPressureSensor_Init(TreatmentPressureSensor *sensor);
/* 周期轮询压力传感器并写入共享采样快照。 */
void TreatmentPressureSensor_Poll(TreatmentPressureSensor *sensor,
                                  TickType_t now,
                                  volatile sensor_data_t *out);

#ifdef __cplusplus
}
#endif

#endif
