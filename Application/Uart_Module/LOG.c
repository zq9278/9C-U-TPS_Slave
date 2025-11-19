/*
 * 模块名称: 轻量日志实现
 * 实现要点:
 *   - LOG/LOG_ISR 将格式化后的字符串封装到 LogMessage_t 并投递到调试口队列。
 *   - 由 Uart_Task.c 的 UartRxParserTask 统一从队列取出并通过 UART 阻塞发送。
 *   - 提供 __io_putchar 以支持 printf 的底层输出到调试口。
 */

#include "LOG.h"
#include <stdarg.h>
#include <stdio.h>
#include "uart_driver.h"

extern UartPort_t debug_uart_port;

/*
 * 函数名称: LOG
 * 功能描述: 线程环境下的日志输出（安全封装 vsnprintf）。
 */
void LOG(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;
    va_start(args, format);
    msg.len = (size_t)vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);
    if (msg.len > LOG_BUF_LEN) msg.len = LOG_BUF_LEN; // 截断保护
    (void)xQueueSend(debug_uart_port.tx_queue, &msg, portMAX_DELAY);
}

/*
 * 函数名称: LOG_ISR
 * 功能描述: 中断环境下的日志输出（使用 FromISR 版本 API）。
 */
void LOG_ISR(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;
    va_start(args, format);
    msg.len = (size_t)vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);

    if (msg.len > LOG_BUF_LEN) msg.len = LOG_BUF_LEN;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(debug_uart_port.tx_queue, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * 函数名称: __io_putchar
 * 功能描述: 重定向 printf 的单字符输出到调试口 UART（阻塞发送）。
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(debug_uart_port.huart, (uint8_t *)&ch, 1, 100);
    return ch;
}

