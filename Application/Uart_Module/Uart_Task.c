/*
 * 模块名称: UART 任务（发送/接收解析/日志输出）
 * 功能概述:
 *   - SerialTxTask: 负责 rk3576 业务口的 DMA 发送调度（按“发送完成”信号量节流）。
 *   - UartRxParserTask: 负责从业务口接收队列取数据进行协议解析，并从调试口日志队列取消息并输出。
 */

#include "Uart_Task.h"
#include "uart_driver.h"
#include "LOG.h"

extern UartPort_t rk3576_uart_port;
extern UartPort_t debug_uart_port;

/*
 * 函数名称: SerialTxTask
 * 功能描述: rk3576 端口的 DMA 发送任务。等待“发送完成”信号量，按队列取出一帧并启动 DMA。
 */
void SerialTxTask(void *argument)
{
    (void)argument;
    UartTxMessage_t msg;
    for (;;)
    {
        if (xSemaphoreTake(rk3576_uart_port.uartTxDoneSem, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(rk3576_uart_port.tx_queue, &msg, portMAX_DELAY) == pdPASS)
            {
                HAL_UART_Transmit_DMA(rk3576_uart_port.huart, (uint8_t *)msg.data, msg.length);
            }
            else
            {
                xSemaphoreGive(rk3576_uart_port.uartTxDoneSem);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

/*
 * 函数名称: UartRxParserTask
 * 功能描述: 解析业务口接收到的数据帧；同时负责调试口日志的直接发送输出。
 */
void UartRxParserTask(void *argument)
{
    (void)argument;
    UartRxMessage_t rx_msg;
    LogMessage_t    log_msg;

    for (;;)
    {
        if (xQueueReceive(rk3576_uart_port.rx_queue, &rx_msg, 0) == pdPASS)
        {
            rk3576_uart_port.parser(rx_msg.data, rx_msg.length);
        }

        if (xQueueReceive(debug_uart_port.tx_queue, &log_msg, 0) == pdPASS)
        {
            HAL_UART_Transmit(debug_uart_port.huart, (uint8_t*)log_msg.buf, (uint16_t)log_msg.len, HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

