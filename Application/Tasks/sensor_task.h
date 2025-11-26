#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "system_app.h"
#include "semphr.h"

void SensorTask(void *argument);
extern SemaphoreHandle_t gRtdDrdySem;

#endif // SENSOR_TASK_H
