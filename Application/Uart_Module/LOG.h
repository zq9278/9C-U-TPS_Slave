#ifndef ELECTRICAL_MUSCLE_QUBEMX_LOG_H
#define ELECTRICAL_MUSCLE_QUBEMX_LOG_H

#include "AppMain.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

#define LOG_BUF_LEN   200
#define LOG_QUEUE_LEN 8

typedef struct {
    char   buf[LOG_BUF_LEN];
    size_t len;
} LogMessage_t;

extern QueueHandle_t logQueue;
extern SemaphoreHandle_t logSemaphore;

void LOG_Init(void);
void LOG(const char *format, ...);
void LOG_ISR(const char *format, ...);
void LOG_Try(const char *format, ...);
void LOG_Raw(const uint8_t *data, size_t len);

#define LOG_I(fmt, ...) LOG("[I] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_W(fmt, ...) LOG("[W] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_E(fmt, ...) LOG("[E] " fmt "\r\n", ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif
