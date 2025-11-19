/*
 * 模块名称: 轻量日志接口（FreeRTOS + UART）
 * 功能概述:
 *   - 提供线程安全的 LOG(...) 与中断态 LOG_ISR(...) 输出接口。
 *   - 发送路径：格式化为 LogMessage_t -> 投递到调试口队列 -> 由任务统一串口输出。
 */

#ifndef ELECTRICAL_MUSCLE_QUBEMX_LOG_H
#define ELECTRICAL_MUSCLE_QUBEMX_LOG_H

#include "AppMain.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

/* 日志等级（预留，当前未细分使用） */
typedef enum {
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

#define LOG_BUF_LEN   200   // 单条日志的最大格式化长度
#define LOG_QUEUE_LEN 5     // 调试口日志队列深度

typedef struct {
    char   buf[LOG_BUF_LEN];
    size_t len;
} LogMessage_t;

extern QueueHandle_t logQueue;
extern SemaphoreHandle_t logSemaphore;

/* 接口说明：
 *   - LOG: 线程环境下调用，内部使用 vsnprintf 格式化并投递队列。
 *   - LOG_ISR: 中断环境下调用，使用 FromISR 版本投递队列。
 */
void LOG_Init(void);
void LOG(const char *format, ...);
void LOG_ISR(const char *format, ...);

/* 便捷宏：自动添加等级前缀并换行 */
#define LOG_I(fmt, ...) LOG("[I] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_W(fmt, ...) LOG("[W] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_E(fmt, ...) LOG("[E] " fmt "\r\n", ##__VA_ARGS__)

#endif // ELECTRICAL_MUSCLE_QUBEMX_LOG_H

