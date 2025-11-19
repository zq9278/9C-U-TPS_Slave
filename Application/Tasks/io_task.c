#include "io_task.h"
#include "main.h"

void IoTask(void *argument)
{
    (void)argument;
    for(;;)
    {
        // LED heartbeat depends on app state (simple blink)
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

