#include "App/System/app_tasks.h"

#include <stddef.h>
#include <string.h>
#include "App/System/system_app.h"
#include "Modules/Log/module_log.h"
#include "Modules/communication/communication.h"
#include "Modules/communication/Protocol/rk3576_protocol.h"
#include "Modules/EyeShield/eye_shield_status.h"
#include "Modules/Pid/pid_debug_protocol.h"
#include "treatment_app_controller.h"
#include "Modules/Sensors/treatment_pressure_sensor.h"
#include "Modules/Sensors/treatment_temperature_sensor.h"
#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "task.h"

#define APP_TASK_STACK_WORDS       356U
#define COMM_RX_TASK_STACK_WORDS   320U
#define COMM_TX_TASK_STACK_WORDS   288U
#define SENSOR_TASK_STACK_WORDS    320U
#define CONTROL_TASK_STACK_WORDS   448U
#define SAFETY_TASK_STACK_WORDS    192U

#define SENSOR_PERIOD_MS           2U
#define CONTROL_PERIOD_MS          2U
#define COMM_RX_PERIOD_MS          1U
#define SAFETY_PERIOD_MS           10U
#define PRESS_TELEMETRY_MS         10U
#define TEMP_TELEMETRY_MS          20U
#define EYE_SHIELD_STATUS_MS       100U
#define PID_DEBUG_TELEMETRY_MS     100U

#define TEMP_MAX_C                 60.0f
#define PRESS_MAX_KPA              450.0f
#define CONTROL_RUNTIME_LOG_ENABLE 0U
#define MODE_1_PULSE_ON_MS         1000.0f
#define MODE_1_PULSE_OFF_MS        1000.0f
#define MODE_2_PULSE_ON_MS         1500.0f
#define MODE_2_PULSE_OFF_MS        1500.0f

static TaskHandle_t s_app_task = NULL;
static TaskHandle_t s_comm_rx_task = NULL;
static TaskHandle_t s_comm_tx_task = NULL;
static TaskHandle_t s_sensor_task = NULL;
static TaskHandle_t s_control_task = NULL;
static TaskHandle_t s_safety_task = NULL;

static SemaphoreHandle_t s_rtd_drdy_sem = NULL;

static void AppTask(void *argument);
static void CommRxTask(void *argument);
static void CommTxTask(void *argument);
static void SensorTask(void *argument);
static void ControlTask(void *argument);
static void SafetyTask(void *argument);
static void OnProtocolFrame(void *context,
                            CommunicationInterfaceId interface_id,
                            const CommunicationFrameView *frame);
static void OnAsciiCommand(void *context,
                           CommunicationChannel channel,
                           const char *line,
                           size_t length);
static void send_sensor_mode(temp_acquire_mode_t mode);
static temp_acquire_mode_t resolve_temperature_mode(const control_config_t *cfg);

static void enqueue_tx_u8(uint16_t frame_id, uint8_t value)
{
    tx_frame_t tx;
    (void)memset(&tx, 0, sizeof(tx));
    tx.frame_id = frame_id;
    tx.type = TX_DATA_UINT8;
    tx.v.u8 = value;
    (void)xQueueSend(gTxQueue, &tx, 0U);
}

static void enqueue_tx_f32(uint16_t frame_id, float value)
{
    tx_frame_t tx;
    (void)memset(&tx, 0, sizeof(tx));
    tx.frame_id = frame_id;
    tx.type = TX_DATA_FLOAT;
    tx.v.f32 = value;
    (void)xQueueSend(gTxQueue, &tx, 0U);
}

static void send_ctrl_command(ctrl_cmd_id_t id, const control_config_t *cfg)
{
    ctrl_cmd_t cmd;
    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.id = id;
    if (cfg != NULL)
    {
        cmd.cfg = *cfg;
    }
    if (xQueueSend(gCtrlCmdQueue, &cmd, 0U) != pdTRUE)
    {
        LOG_E("ctrl queue full cmd=%u", (unsigned int)id);
    }
}

static void send_pid_ctrl_command(const PidDebugCommand *pid_cmd)
{
    ctrl_cmd_t cmd;

    if (pid_cmd == NULL)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.id = pid_cmd->id;
    cmd.pid_target = pid_cmd->target;
    cmd.kp = pid_cmd->kp;
    cmd.ki = pid_cmd->ki;
    cmd.kd = pid_cmd->kd;
    cmd.enabled = pid_cmd->enabled;

    if (xQueueSend(gCtrlCmdQueue, &cmd, 0U) != pdTRUE)
    {
        LOG_E("ctrl queue full pid cmd=%u", (unsigned int)cmd.id);
    }
}

static void apply_mode_profile(control_config_t *cfg, uint8_t mode)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->mode = (mode <= 1U) ? 1U : 2U;
    if (cfg->mode == 1U)
    {
        cfg->pulse_on_ms = MODE_1_PULSE_ON_MS;
        cfg->pulse_off_ms = MODE_1_PULSE_OFF_MS;
    }
    else
    {
        cfg->pulse_on_ms = MODE_2_PULSE_ON_MS;
        cfg->pulse_off_ms = MODE_2_PULSE_OFF_MS;
    }
}

static void send_sensor_mode(temp_acquire_mode_t mode)
{
    sensor_cmd_t cmd;

    if (gSensorCmdQueue == NULL)
    {
        return;
    }

    cmd.mode = mode;
    (void)xQueueOverwrite(gSensorCmdQueue, &cmd);
}

static void config_defaults(control_config_t *cfg)
{
    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->temp_target = 42.0f;
    cfg->press_target_max = 30.0f;
    cfg->t1_rise_s = TREATMENT_DEFAULT_RISE_S;
    cfg->t2_hold_s = TREATMENT_DEFAULT_HOLD_S;
    cfg->t3_pulse_s = TREATMENT_DEFAULT_PULSE_S;
    apply_mode_profile(cfg, 1U);
    cfg->press_enable_L = 1U;
    cfg->press_enable_R = 1U;
    cfg->treatment_minutes = 1U;
}

static temp_acquire_mode_t resolve_temperature_mode(const control_config_t *cfg)
{
    const uint8_t left_enabled = (uint8_t)((cfg != NULL) &&
                                           (cfg->press_enable_L != 0U) &&
                                           (gSensorData.heaterPresentL != 0U));
    const uint8_t right_enabled = (uint8_t)((cfg != NULL) &&
                                            (cfg->press_enable_R != 0U) &&
                                            (gSensorData.heaterPresentR != 0U));

    if ((left_enabled != 0U) && (right_enabled != 0U))
    {
        return TEMP_ACQUIRE_MODE_DUAL_SCAN;
    }
    if (left_enabled != 0U)
    {
        return TEMP_ACQUIRE_MODE_LEFT_FIXED;
    }
    if (right_enabled != 0U)
    {
        return TEMP_ACQUIRE_MODE_RIGHT_FIXED;
    }
    return TEMP_ACQUIRE_MODE_IDLE;
}

static void OnProtocolFrame(void *context,
                            CommunicationInterfaceId interface_id,
                            const CommunicationFrameView *frame)
{
    app_cmd_t cmd;
    (void)context;

    if ((frame == NULL) ||
        (gAppCommandQueue == NULL) ||
        (interface_id != COMM_INTERFACE_RK3576_UART3))
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));

    if (Rk3576Protocol_FrameToAppCommand(frame, &cmd) == 0U)
    {
        return;
    }

    if (frame->frame_id == PROTOCOL_ID_U8_START_TREATMENT)
    {
        LOG_I("rx start frame");
    }
    else if (frame->frame_id == PROTOCOL_ID_U8_STOP_TREATMENT)
    {
        LOG_I("rx stop frame");
    }

    if (xQueueSend(gAppCommandQueue, &cmd, 0U) != pdTRUE)
    {
        LOG_E("app queue full frame=0x%04X cmd=%u",
              (unsigned int)frame->frame_id,
              (unsigned int)cmd.id);
    }
}

static void OnAsciiCommand(void *context,
                           CommunicationChannel channel,
                           const char *line,
                           size_t length)
{
    PidDebugCommand cmd;
    (void)context;

    if ((channel != COMM_CHANNEL_UART1) || (line == NULL))
    {
        return;
    }

    if (PidDebugProtocol_ParseCommand(line, length, &cmd) == 0U)
    {
        return;
    }

    send_pid_ctrl_command(&cmd);
}

static void AppTask(void *argument)
{
    app_cmd_t cmd;
    control_config_t cfg;
    uint8_t paused = 0U;

    (void)argument;
    config_defaults(&cfg);
    send_sensor_mode(TEMP_ACQUIRE_MODE_IDLE);
    send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);

    for (;;)
    {
        if (xQueueReceive(gAppCommandQueue, &cmd, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        switch (cmd.id)
        {
        case APP_CMD_HEARTBEAT:
            enqueue_tx_u8(PROTOCOL_ID_U8_HEARTBEAT_ACK, 1U);
            break;
        case APP_CMD_SET_PRESSURE_KPA:
            cfg.press_target_max = cmd.v.f32;
            if (cfg.press_target_max < 0.0f) { cfg.press_target_max = 0.0f; }
            if (cfg.press_target_max > 400.0f) { cfg.press_target_max = 400.0f; }
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_SET_TEMP:
            cfg.temp_target = cmd.v.f32;
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_LEFT_ENABLE:
            cfg.press_enable_L = (cmd.v.u8 != 0U) ? 1U : 0U;
            if (cfg.running != 0U)
            {
                send_sensor_mode(resolve_temperature_mode(&cfg));
            }
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_RIGHT_ENABLE:
            cfg.press_enable_R = (cmd.v.u8 != 0U) ? 1U : 0U;
            if (cfg.running != 0U)
            {
                send_sensor_mode(resolve_temperature_mode(&cfg));
            }
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_LEFT_HEATER_FUSE_BLOW:
            EyeShieldStatus_RequestFuseBlow(1U, 0U);
            break;
        case APP_CMD_RIGHT_HEATER_FUSE_BLOW:
            EyeShieldStatus_RequestFuseBlow(0U, 1U);
            break;
        case APP_CMD_SET_TREATMENT_TIME:
            cfg.treatment_minutes = (cmd.v.u16 == 0U) ? 1U : cmd.v.u16;
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_MODE_SELECT:
            apply_mode_profile(&cfg, cmd.v.u8);
            LOG_I("app mode=%u pulse_on=%d pulse_off=%d",
                  cfg.mode,
                  (int)cfg.pulse_on_ms,
                  (int)cfg.pulse_off_ms);
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_START:
            if ((cfg.press_enable_L == 0U) && (cfg.press_enable_R == 0U))
            {
                cfg.press_enable_L = 1U;
                cfg.press_enable_R = 1U;
            }
            paused = 0U;
            cfg.running = 1U;
            gTreatmentRunning = 1U;
            LOG_I("app start temp=%d.%02d press=%d L_en=%u R_en=%u",
                  (int)cfg.temp_target,
                  (int)((cfg.temp_target - (float)((int)cfg.temp_target)) * 100.0f),
                  (int)cfg.press_target_max,
                  cfg.press_enable_L,
                  cfg.press_enable_R);
            send_sensor_mode(resolve_temperature_mode(&cfg));
            send_ctrl_command(CTRL_CMD_START, &cfg);
            break;
        case APP_CMD_STOP:
            paused = 0U;
            cfg.running = 0U;
            gTreatmentRunning = 0U;
            LOG_I("app stop");
            send_sensor_mode(TEMP_ACQUIRE_MODE_IDLE);
            send_ctrl_command(CTRL_CMD_STOP, &cfg);
            break;
        case APP_CMD_PAUSE_RESUME:
            if (cmd.v.u8 == 0U)
            {
                paused = 1U;
                gTreatmentRunning = 0U;
                LOG_I("app pause");
                send_sensor_mode(TEMP_ACQUIRE_MODE_IDLE);
                send_ctrl_command(CTRL_CMD_PAUSE, &cfg);
            }
            else
            {
                paused = 0U;
                gTreatmentRunning = (cfg.running != 0U) ? 1U : 0U;
                LOG_I("app resume");
                send_sensor_mode(resolve_temperature_mode(&cfg));
                send_ctrl_command(CTRL_CMD_RESUME, &cfg);
            }
            (void)paused;
            break;
        case APP_CMD_SAVE_PARAM:
        case APP_CMD_NONE:
        default:
            break;
        }
    }
}

static void CommRxTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        Communication_PollRx();
        vTaskDelay(pdMS_TO_TICKS(COMM_RX_PERIOD_MS));
    }
}

static void CommTxTask(void *argument)
{
    tx_frame_t tx;
    (void)argument;

    for (;;)
    {
        if (xQueueReceive(gTxQueue, &tx, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        switch (tx.type)
        {
        case TX_DATA_UINT8:
            (void)Communication_SendU8(tx.frame_id, tx.v.u8);
            break;
        case TX_DATA_FLOAT:
            (void)Communication_SendF32(tx.frame_id, tx.v.f32);
            break;
        case TX_DATA_U16:
            (void)Communication_SendU16(tx.frame_id, tx.v.u16);
            break;
        case TX_DATA_U32:
            (void)Communication_SendU32(tx.frame_id, tx.v.u32);
            break;
        case TX_DATA_TEXT:
        default:
            (void)Communication_SendText(tx.frame_id, tx.v.text);
            break;
        }
    }
}

static void SensorTask(void *argument)
{
    TreatmentTemperatureSensor temperature_sensor;
    TreatmentPressureSensor pressure_sensor;
    sensor_cmd_t sensor_cmd;
    TickType_t now;

    (void)argument;

    s_rtd_drdy_sem = xSemaphoreCreateBinary();
    TreatmentTemperatureSensor_Init(&temperature_sensor, s_rtd_drdy_sem);
    TreatmentPressureSensor_Init(&pressure_sensor);

    for (;;)
    {
        while ((gSensorCmdQueue != NULL) &&
               (xQueueReceive(gSensorCmdQueue, &sensor_cmd, 0U) == pdTRUE))
        {
            TreatmentTemperatureSensor_SetMode(&temperature_sensor, sensor_cmd.mode);
        }

        now = xTaskGetTickCount();
        TreatmentTemperatureSensor_Poll(&temperature_sensor, &gSensorData);
        TreatmentPressureSensor_Poll(&pressure_sensor, now, &gSensorData);
        gSensorData.tick = now;
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

static void ControlTask(void *argument)
{
    TreatmentAppController controller;
    TreatmentAppRuntime runtime;
    ctrl_cmd_t cmd;
    TickType_t now;
    TickType_t next_press_tx;
    TickType_t next_temp_tx;
    TickType_t next_eye_shield_status;
    TickType_t next_control_log;
    TickType_t next_pid_debug_tx;

    (void)argument;
    TreatmentAppController_Init(&controller);
    EyeShieldStatus_Init();
    now = xTaskGetTickCount();
    next_press_tx = now + pdMS_TO_TICKS(PRESS_TELEMETRY_MS);
    next_temp_tx = now + pdMS_TO_TICKS(TEMP_TELEMETRY_MS);
    next_eye_shield_status = now + pdMS_TO_TICKS(EYE_SHIELD_STATUS_MS);
    next_control_log = now + pdMS_TO_TICKS(1000U);
    next_pid_debug_tx = now + pdMS_TO_TICKS(PID_DEBUG_TELEMETRY_MS);

    for (;;)
    {
        now = xTaskGetTickCount();

        while (xQueueReceive(gCtrlCmdQueue, &cmd, 0U) == pdTRUE)
        {
            TreatmentAppController_HandleCommand(&controller, &cmd, now);
        }

        EyeShieldStatus_Service();
        if ((int32_t)(now - next_eye_shield_status) >= 0)
        {
            next_eye_shield_status = now + pdMS_TO_TICKS(EYE_SHIELD_STATUS_MS);
            EyeShieldStatus_Process(&controller.cfg);
        }

        TreatmentAppController_Run(&controller,&gSensorData,now,(float)CONTROL_PERIOD_MS / 1000.0f, &runtime);

#if CONTROL_RUNTIME_LOG_ENABLE
        if ((runtime.session_active != 0U) &&
            ((int32_t)(now - next_control_log) >= 0))
        {
            next_control_log = now + pdMS_TO_TICKS(1000U);
            LOG_I("ctrl phase=%c run=%u out=%u L_en=%u R_en=%u tL=%d.%02d tR=%d.%02d pwmL=%u pwmR=%u",
                  runtime.phase_char,
                  controller.cfg.running,
                  runtime.running_outputs,
                  controller.cfg.press_enable_L,
                  controller.cfg.press_enable_R,
                  (int)gSensorData.tempL,
                  (int)((gSensorData.tempL - (float)((int)gSensorData.tempL)) * 100.0f),
                  (int)gSensorData.tempR,
                  (int)((gSensorData.tempR - (float)((int)gSensorData.tempR)) * 100.0f),
                  runtime.heat_left_pwm,
                  runtime.heat_right_pwm);
        }
#else
        (void)next_control_log;
#endif

        if ((runtime.session_active != 0U) &&
            ((int32_t)(now - next_press_tx) >= 0))
        {
            next_press_tx = now + pdMS_TO_TICKS(PRESS_TELEMETRY_MS);
            enqueue_tx_f32(PROTOCOL_ID_F32_LEFT_PRESSURE_VALUE, gSensorData.pressL);
        }

        if ((runtime.session_active != 0U) &&
            ((int32_t)(now - next_temp_tx) >= 0))
        {
            next_temp_tx = now + pdMS_TO_TICKS(TEMP_TELEMETRY_MS);
            enqueue_tx_f32(PROTOCOL_ID_F32_LEFT_TEMP_VALUE, gSensorData.tempL);
            enqueue_tx_f32(PROTOCOL_ID_F32_RIGHT_TEMP_VALUE, gSensorData.tempR);
            enqueue_tx_u8(PROTOCOL_ID_U8_MODE_CURVES, runtime.phase_char);
        }

        if ((controller.pid_debug_stream_enabled != 0U) &&
            ((int32_t)(now - next_pid_debug_tx) >= 0))
        {
            next_pid_debug_tx = now + pdMS_TO_TICKS(PID_DEBUG_TELEMETRY_MS);
            (void)PidDebugProtocol_SendSample(PID_DEBUG_TARGET_HEAT_LEFT,
                                              (uint32_t)(now * portTICK_PERIOD_MS),
                                              &controller.heat_left_pid);
            (void)PidDebugProtocol_SendSample(PID_DEBUG_TARGET_HEAT_RIGHT,
                                              (uint32_t)(now * portTICK_PERIOD_MS),
                                              &controller.heat_right_pid);
            (void)PidDebugProtocol_SendSample((pid_debug_target_t)runtime.pid_stage,
                                              (uint32_t)(now * portTICK_PERIOD_MS),
                                              &controller.pressure_pid);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void SafetyTask(void *argument)
{
    uint8_t fault_active = 0U;
    (void)argument;

    for (;;)
    {
        uint8_t fault =
            (uint8_t)((gSensorData.tempL > TEMP_MAX_C) ||
                      (gSensorData.tempR > TEMP_MAX_C) ||
                      (gSensorData.pressL > PRESS_MAX_KPA) ||
                      (gSensorData.pressR > PRESS_MAX_KPA));

        if ((fault != 0U) && (fault_active == 0U))
        {
            LOG_E("safety stop tempL=%d.%02d tempR=%d.%02d pressL=%d.%02d pressR=%d.%02d",
                  (int)gSensorData.tempL,
                  (int)((gSensorData.tempL - (float)((int)gSensorData.tempL)) * 100.0f),
                  (int)gSensorData.tempR,
                  (int)((gSensorData.tempR - (float)((int)gSensorData.tempR)) * 100.0f),
                  (int)gSensorData.pressL,
                  (int)((gSensorData.pressL - (float)((int)gSensorData.pressL)) * 100.0f),
                  (int)gSensorData.pressR,
                  (int)((gSensorData.pressR - (float)((int)gSensorData.pressR)) * 100.0f));
            send_ctrl_command(CTRL_CMD_STOP, NULL);
            gTreatmentRunning = 0U;
            fault_active = 1U;
        }
        else if (fault == 0U)
        {
            fault_active = 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}

void AppTasks_Init(void)
{
    CommunicationCallbacks callbacks;
    (void)memset(&callbacks, 0, sizeof(callbacks));
    callbacks.on_protocol_frame = OnProtocolFrame;
    callbacks.on_ascii_command = OnAsciiCommand;
    Communication_SetCallbacks(&callbacks);

    (void)xTaskCreate(AppTask, "app", APP_TASK_STACK_WORDS, NULL,tskIDLE_PRIORITY + 3U, &s_app_task);
    (void)xTaskCreate(CommRxTask, "comm_rx", COMM_RX_TASK_STACK_WORDS, NULL,tskIDLE_PRIORITY + 4U, &s_comm_rx_task);
    (void)xTaskCreate(CommTxTask, "comm_tx", COMM_TX_TASK_STACK_WORDS, NULL,tskIDLE_PRIORITY + 3U, &s_comm_tx_task);
    (void)xTaskCreate(SensorTask, "sensor", SENSOR_TASK_STACK_WORDS, NULL,tskIDLE_PRIORITY + 4U, &s_sensor_task);
    (void)xTaskCreate(ControlTask, "control", CONTROL_TASK_STACK_WORDS, NULL,tskIDLE_PRIORITY + 5U, &s_control_task);
    (void)xTaskCreate(SafetyTask, "safety", SAFETY_TASK_STACK_WORDS, NULL,tskIDLE_PRIORITY + 6U, &s_safety_task);

    (void)s_app_task;
    (void)s_comm_rx_task;
    (void)s_comm_tx_task;
    (void)s_sensor_task;
    (void)s_control_task;
    (void)s_safety_task;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == RTD_RDY_Pin) && (s_rtd_drdy_sem != NULL))
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        (void)xSemaphoreGiveFromISR(s_rtd_drdy_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
