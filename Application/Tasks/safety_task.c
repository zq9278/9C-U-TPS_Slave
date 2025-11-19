#include "safety_task.h"
#include "LOG.h"

// Simple thresholds (can be tuned or loaded from settings)
#define TEMP_MAX_C     60.0f
#define PRESS_MAX_KPA  40.0f

void SafetyTask(void *argument)
{
    (void)argument;
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint8_t fault = 0;
        if (gSensorData.tempL > TEMP_MAX_C || gSensorData.tempR > TEMP_MAX_C) fault = 1;
        if (gSensorData.pressL > PRESS_MAX_KPA || gSensorData.pressR > PRESS_MAX_KPA) fault = 1;
        if (fault) {
            xQueueSend(gSafetyQueue, &fault, 0);
        }
    }
}

