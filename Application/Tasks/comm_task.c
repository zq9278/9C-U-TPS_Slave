#include <string.h>
#include "comm_task.h"
#include "uart_driver.h"
#include "Uart_Communicate.h"
#include "LOG.h"

extern UartPort_t rk3576_uart_port;
extern UartPort_t debug_uart_port;

void CommTask(void *argument)
{
    (void)argument;
    rk3576_uart_port_Init(&rk3576_uart_port);
    debug_uart_port_Init(&debug_uart_port);
    UartRxMessage_t rx_msg;
    tx_frame_t tx;
    LogMessage_t log_msg;

    for(;;)
    {
        // RX path: parse incoming business port data
        if (xQueueReceive(rk3576_uart_port.rx_queue, &rx_msg, 0) == pdPASS) {
            parse_rk3576_uart_port_stream(rx_msg.data, rx_msg.length);
        }

        // TX path: send frames enqueued by other tasks
        if (xQueueReceive(gTxQueue, &tx, 0) == pdPASS) {
            switch (tx.type) {
                case TX_DATA_FLOAT:
                    rk3576_uart_port.sender(DATA_FLOAT, tx.frame_id, &tx.v.f32);
                    break;
                case TX_DATA_UINT8:
                    rk3576_uart_port.sender(DATA_UINT8_T, tx.frame_id, &tx.v.u8);
                    break;
                case TX_DATA_U16:
                    rk3576_uart_port.sender(DATA_UINT16_T, tx.frame_id, &tx.v.u16);
                    break;
                case TX_DATA_U32:
                    rk3576_uart_port.sender(DATA_UINT32_T, tx.frame_id, &tx.v.u32);
                    break;
                case TX_DATA_TEXT:
                default:
                    rk3576_uart_port.sender(DATA_TYPE_TEXT, tx.frame_id, tx.v.text);
                    break;
            }
        }

        // Debug log drain
        if (xQueueReceive(debug_uart_port.tx_queue, &log_msg, 0) == pdPASS) {
            HAL_UART_Transmit(debug_uart_port.huart, (uint8_t*)log_msg.buf, (uint16_t)log_msg.len, HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

