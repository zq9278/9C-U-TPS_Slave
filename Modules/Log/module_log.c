#include "Modules/Log/module_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"

/* 统一日志格式：[Level] message\r\n */
void ModuleLog_Printf(const char *level, const char *fmt, ...)
{
    char buffer[160];
    int length;
    va_list args;

    if ((level == NULL) || (fmt == NULL))
    {
        return;
    }

    length = snprintf(buffer, sizeof(buffer), "[%s] ", level);
    if (length < 0)
    {
        return;
    }

    va_start(args, fmt);
    length += vsnprintf(&buffer[length], sizeof(buffer) - (size_t)length, fmt, args);
    va_end(args);

    if (length < 0)
    {
        return;
    }

    if ((size_t)length > (sizeof(buffer) - 3U))
    {
        length = (int)(sizeof(buffer) - 3U);
    }
    buffer[length++] = '\r';
    buffer[length++] = '\n';
    buffer[length] = '\0';

    Communication_WriteLog(buffer);
}

int __io_putchar(int ch)
{
    uint8_t data = (uint8_t)ch;

    /* 优先走通信模块日志接口；若通信尚未初始化，则回退为 HAL 阻塞发送。 */
    if (Communication_WriteLogBuffer(&data, 1U) == 1)
    {
        return ch;
    }

    (void)HAL_UART_Transmit(&huart1, &data, 1U, 2U);
    return ch;
}
