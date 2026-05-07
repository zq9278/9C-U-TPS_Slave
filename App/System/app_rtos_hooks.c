#include "FreeRTOS.h"
#include "task.h"
#include "TreatmentActuators.h"

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    TreatmentActuators_SetIdle();
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    TreatmentActuators_SetIdle();
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
