/*
 * 模块名称: UART 任务声明
 * 功能概述:
 *   - 声明发送任务与接收/日志处理任务，用于在 FreeRTOS 中创建并运行。
 */

#ifndef UART_TASK_H
#define UART_TASK_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "uart_driver.h"

/* UART 发送任务（DMA 调度） */
void SerialTxTask(void *argument);

/* UART 接收解析 + 日志输出任务 */
void UartRxParserTask(void *argument);

#endif // UART_TASK_H

