#include "LOG.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "uart_driver.h"

extern UartPort_t debug_uart_port;

void LOG(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;

    va_start(args, format);
    msg.len = (size_t)vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);

    if (msg.len > LOG_BUF_LEN) {
        msg.len = LOG_BUF_LEN;
    }

    (void)xQueueSend(debug_uart_port.tx_queue, &msg, portMAX_DELAY);
}

void LOG_ISR(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;

    va_start(args, format);
    msg.len = (size_t)vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);

    if (msg.len > LOG_BUF_LEN) {
        msg.len = LOG_BUF_LEN;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(debug_uart_port.tx_queue, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LOG_Try(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;

    va_start(args, format);
    msg.len = (size_t)vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);

    if (msg.len > LOG_BUF_LEN) {
        msg.len = LOG_BUF_LEN;
    }

    (void)xQueueSend(debug_uart_port.tx_queue, &msg, 0);
}

void LOG_Raw(const uint8_t *data, size_t len)
{
    LogMessage_t msg;

    if (data == NULL) {
        return;
    }
    if (len > LOG_BUF_LEN) {
        len = LOG_BUF_LEN;
    }

    memcpy(msg.buf, data, len);
    msg.len = len;
    (void)xQueueSend(debug_uart_port.tx_queue, &msg, 0);
}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(debug_uart_port.huart, (uint8_t *)&ch, 1, 100);
    return ch;
}
