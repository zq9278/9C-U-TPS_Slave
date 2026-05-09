#include "FreeRTOS.h"
#include "task.h"

#include "treatment_app_controller.h"

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    TreatmentAppController_SetSystemIdleOutputs();
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    TreatmentAppController_SetSystemIdleOutputs();
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
