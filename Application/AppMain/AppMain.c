// Created by zq on 2025/5/21.

#include "AppMain.h"
#include "SYS.h"
#include "uart_driver.h"
#include "Uart_Communicate.h"
#include "Uart_Task.h"
#include "system_app.h"

// FreeRTOS native API
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// App dependencies used by tasks
#include <string.h>
#include "main.h"
#include "heat.h"
#include "ads1248.h"
#include "Pressure_sensor.h"
#include "apply.h"
#include "ds18b20.h"
#include "water.h"
#include "Config/config.h"


// ---------------- FreeRTOS native tasks (migrated from Core/Src/app_freertos.c) ----------------

// (legacy globals removed)

extern UartPort_t rk3576_uart_port;
extern UartPort_t debug_uart_port;

// Queues and shared state
QueueHandle_t gCmdQueue = NULL;
QueueHandle_t gCtrlCmdQueue = NULL;
QueueHandle_t gTxQueue = NULL;
QueueHandle_t gStorageQueue = NULL;
QueueHandle_t gSafetyQueue = NULL;

volatile sensor_data_t gSensorData = {0};
volatile app_state_t gAppState = APP_STATE_RUN_MODE1;//自定义曲线模式

// Task prototypes
// legacy task prototypes removed

void AppMain_FreeRTOS_Init(void)
{
    // Init UART ports (Comm on USART2, Debug on USART1)
   
   

    // Create queues
    gCmdQueue      = xQueueCreate(16, sizeof(app_cmd_t));
    gCtrlCmdQueue  = xQueueCreate(8, sizeof(ctrl_cmd_t));
    gTxQueue       = xQueueCreate(16, sizeof(tx_frame_t));
    gStorageQueue  = xQueueCreate(4, sizeof(storage_cmd_t));
    gSafetyQueue   = xQueueCreate(8, sizeof(uint8_t));

    configASSERT(gCmdQueue && gCtrlCmdQueue && gTxQueue && gStorageQueue && gSafetyQueue);

    // Create tasks with priorities
    BaseType_t ret;
    ret = xTaskCreate(CommTask,    "CommTask",    384, NULL, 21, NULL);
    configASSERT(ret == pdPASS);
    ret = xTaskCreate(AppTask,     "AppTask",     512, NULL, 22, NULL);
    configASSERT(ret == pdPASS);
    ret = xTaskCreate(SensorTask,  "SensorTask",  384, NULL, 24, NULL);
    configASSERT(ret == pdPASS);
    ret = xTaskCreate(ControlTask, "ControlTask", 512, NULL, 25, NULL);
    configASSERT(ret == pdPASS);
    ret = xTaskCreate(StorageTask, "StorageTask", 384, NULL, 5,  NULL);
    configASSERT(ret == pdPASS);
    ret = xTaskCreate(IoTask,      "IoTask",      256, NULL, 3,  NULL);
    configASSERT(ret == pdPASS);
    ret = xTaskCreate(SafetyTask,  "SafetyTask",  384, NULL, 20, NULL);
    configASSERT(ret == pdPASS);

    // Kick storage load on boot
    storage_cmd_t cmd = STORAGE_CMD_LOAD_ALL;
    xQueueSend(gStorageQueue, &cmd, 0);
}
