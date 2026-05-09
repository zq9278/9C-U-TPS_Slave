#include "App/System/app_main.h"

#include <string.h>
#include "App/System/app_tasks.h"
#include "App/System/system_app.h"
#include "Modules/communication/communication.h"
#include "UserDrivers/user_drivers_board.h"

/* 由传感器任务单写、其他任务多读的共享采样快照。 */
volatile sensor_data_t gSensorData;
/* 对外暴露的治疗运行标志，主要供其他任务/模块快速判断当前是否处于治疗态。 */
volatile uint8_t gTreatmentRunning = 0U;

/* RK3576 协议命令入口队列：CommRxTask/AppTask 使用。 */
QueueHandle_t gAppCommandQueue = NULL;
/* 控制命令队列：AppTask 向 ControlTask 下发控制意图。 */
QueueHandle_t gCtrlCmdQueue = NULL;
/* 传感器模式队列：AppTask 向 SensorTask 下发温度采样策略。 */
QueueHandle_t gSensorCmdQueue = NULL;
/* 统一发送队列：各任务把待上报帧压入此队列，由 CommTxTask 串行发送到 UART3。 */
QueueHandle_t gTxQueue = NULL;

void AppMain_FreeRTOS_Init(void)
{
    /* 启动前先清空共享采样结构，避免首次上报旧内存中的随机值。 */
    (void)memset((void *)&gSensorData, 0, sizeof(gSensorData));

    /*
     * 统一在应用入口创建全部关键队列，便于后续模块通过 extern 直接引用。
     * 队列深度依据当前任务负载和协议吞吐量折中设定。
     */
    gAppCommandQueue = xQueueCreate(8U, sizeof(app_cmd_t));
    gCtrlCmdQueue = xQueueCreate(5U, sizeof(ctrl_cmd_t));
    gSensorCmdQueue = xQueueCreate(1U, sizeof(sensor_cmd_t));
    gTxQueue = xQueueCreate(12U, sizeof(tx_frame_t));

    /* 先拉起底层板级资源，再初始化通信和任务。 */
    UserDrivers_BoardInit();
    Communication_Init();
    AppTasks_Init();
}
