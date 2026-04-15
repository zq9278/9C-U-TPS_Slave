#include "FreeRTOS.h"
#include "task.h"
#include "system_app.h"
#include "uart_driver.h"
#include "LOG.h"

extern UartPort_t debug_uart_port;

void DebugLogTask(void *argument)
{
    LogMessage_t log_msg;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(debug_uart_port.tx_queue, &log_msg, portMAX_DELAY) == pdPASS) {
            HAL_UART_Transmit(debug_uart_port.huart,
                              (uint8_t *)log_msg.buf,
                              (uint16_t)log_msg.len,
                              100);
        }
    }
}
