#include "App/System/app_main.h"

#include <string.h>
#include "App/System/app_tasks.h"
#include "App/System/system_app.h"
#include "Modules/communication/communication.h"
#include "UserDrivers/user_drivers_board.h"

volatile sensor_data_t gSensorData;
volatile uint8_t gTreatmentRunning = 0U;

QueueHandle_t gAppCommandQueue = NULL;
QueueHandle_t gCtrlCmdQueue = NULL;
QueueHandle_t gSensorCmdQueue = NULL;
QueueHandle_t gTxQueue = NULL;

void AppMain_FreeRTOS_Init(void)
{
    (void)memset((void *)&gSensorData, 0, sizeof(gSensorData));

    gAppCommandQueue = xQueueCreate(8U, sizeof(app_cmd_t));
    gCtrlCmdQueue = xQueueCreate(5U, sizeof(ctrl_cmd_t));
    gSensorCmdQueue = xQueueCreate(1U, sizeof(sensor_cmd_t));
    gTxQueue = xQueueCreate(12U, sizeof(tx_frame_t));

    UserDrivers_BoardInit();
    Communication_Init();
    AppTasks_Init();
}
