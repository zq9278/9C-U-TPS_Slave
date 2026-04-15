#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
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

    for(;;)
    {
        // RX path: parse incoming business port data
        if (xQueueReceive(rk3576_uart_port.rx_queue, &rx_msg, 0) == pdPASS) {
            parse_rk3576_uart_port_stream(rx_msg.data, rx_msg.length);
        }

        // TX path: drain all frames enqueued by other tasks
        while (xQueueReceive(gTxQueue, &tx, 0) == pdPASS) {
            //LOG_I("comm send: type=%u frame_id=0x%04X", tx.type, tx.frame_id);
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

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
