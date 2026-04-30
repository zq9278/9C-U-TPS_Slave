#include "FreeRTOS.h"
#include "task.h"

volatile const char *gStackOverflowTaskName = 0;
volatile void *gStackOverflowTaskHandle = 0;
volatile uint8_t gMallocFailedHookHit = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    gStackOverflowTaskHandle = (void *)xTask;
    gStackOverflowTaskName = pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    gMallocFailedHookHit = 1U;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
