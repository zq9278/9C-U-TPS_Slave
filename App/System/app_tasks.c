#include "App/System/app_tasks.h"

#include <stddef.h>
#include <string.h>
#define MODULE_LOG_ENABLED MODULE_LOG_APP_TASK_ENABLE
#include "App/System/system_app.h"
#include "Modules/Log/module_log.h"
#include "Modules/communication/communication.h"
#include "Modules/communication/Protocol/pid_debug_protocol.h"
#include "Modules/communication/Protocol/rk3576_protocol.h"
#include "Modules/EyeShield/eye_shield_status.h"
#include "Modules/Filter/target_window_filter.h"
#include "Modules/Heat/treatment_heating_control.h"
#include "treatment_app_controller.h"
#include "Modules/Sensors/treatment_pressure_sensor.h"
#include "Modules/Sensors/treatment_temperature_sensor.h"
#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"
#include "task.h"

/*
 * 本文件是应用层任务编排中心：
 * - AppTask 负责把协议命令翻译为控制配置；
 * - CommRxTask / CommTxTask 负责业务串口收发；
 * - SensorTask 负责压力/温度采样；
 * - ControlTask 负责压力波形、加热闭环和遥测；
 * - SafetyTask 负责超温/超压保护。
 *
 * 整体数据流为：
 * RK3576 帧 -> gAppCommandQueue -> AppTask -> gCtrlCmdQueue -> ControlTask
 *                                        \-> gSensorCmdQueue -> SensorTask
 * 其他任务上报 -> gTxQueue -> CommTxTask -> UART3 -> RK3576
 */

/* 各任务栈深，单位为 StackType_t 个数。 */
#define APP_TASK_STACK_WORDS       356U
#define COMM_RX_TASK_STACK_WORDS   320U
#define COMM_TX_TASK_STACK_WORDS   288U
#define SENSOR_TASK_STACK_WORDS    320U
#define CONTROL_TASK_STACK_WORDS   448U
#define SAFETY_TASK_STACK_WORDS    192U

/* 各任务周期/节拍宏，集中定义便于统一调整。 */
#define SENSOR_PERIOD_MS           2U
#define CONTROL_PERIOD_MS          2U
#define COMM_RX_PERIOD_MS          1U
#define SAFETY_PERIOD_MS           10U
#define PRESS_TELEMETRY_MS         10U
#define TEMP_TELEMETRY_MS          20U
#define EYE_SHIELD_STATUS_MS       100U
#define PID_DEBUG_TELEMETRY_MS     100U
/* 是否启用“发送给 RK3576 的遥测值”窗口滤波。1=启用，0=直接发送原始采样值。 */
#define RK3576_TELEMETRY_FILTER_ENABLE 1U
#define RK3576_PRESS_FILTER_WINDOW_MS PRESS_TELEMETRY_MS
#define RK3576_TEMP_FILTER_WINDOW_MS  TEMP_TELEMETRY_MS
#define RK3576_PRESS_FILTER_SAMPLES ((uint16_t)(RK3576_PRESS_FILTER_WINDOW_MS / CONTROL_PERIOD_MS))
#define RK3576_TEMP_FILTER_SAMPLES  ((uint16_t)(RK3576_TEMP_FILTER_WINDOW_MS / CONTROL_PERIOD_MS))

#define TEMP_MAX_C                 50.0f
#define PRESS_MAX_KPA              450.0f
#define CONTROL_RUNTIME_LOG_ENABLE 0U
/* RK3576 下发的三档模式值。 */
#define MODE_SELECT_RELAX          1U
#define MODE_SELECT_STANDARD       2U
#define MODE_SELECT_FAST           3U
#define MODE_RELAX_PULSE_ON_MS     2000.0f
#define MODE_RELAX_PULSE_OFF_MS    2000.0f
#define MODE_STANDARD_PULSE_ON_MS  1500.0f
#define MODE_STANDARD_PULSE_OFF_MS 1500.0f
#define MODE_FAST_PULSE_ON_MS      1000.0f
#define MODE_FAST_PULSE_OFF_MS     1000.0f

static TaskHandle_t s_app_task = NULL;
static TaskHandle_t s_comm_rx_task = NULL;
static TaskHandle_t s_comm_tx_task = NULL;
static TaskHandle_t s_sensor_task = NULL;
static TaskHandle_t s_control_task = NULL;
static TaskHandle_t s_safety_task = NULL;

/* ADS1248 数据就绪中断通过该信号量驱动温度采样。 */
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
static void OnLogRx(void *context,
                    CommunicationChannel channel,
                    const uint8_t *data,
                    size_t length);
static void send_sensor_mode(temp_acquire_mode_t mode, uint8_t suppress_rtd_fail_log);
static temp_acquire_mode_t resolve_temperature_mode(const control_config_t *cfg);
static uint8_t resolve_temperature_log_suppress(const control_config_t *cfg);
static void request_app_stop(void);
static void set_pending_stop_reason(stop_reason_t reason);
static uint8_t consume_pending_stop_reason(stop_reason_t *reason);
static void Rk3576TelemetryFilter_Init(void *state, TickType_t now_tick);
static void Rk3576TelemetryFilter_Reset(void *state, TickType_t now_tick);
static void Rk3576TelemetryFilter_Update(void *state,
                                         const volatile sensor_data_t *sensor,
                                         TickType_t now_tick);
static float Rk3576TelemetryFilter_GetPressureLeft(const void *state,
                                                   const volatile sensor_data_t *sensor);
static float Rk3576TelemetryFilter_GetTempLeft(const void *state,
                                               const volatile sensor_data_t *sensor);
static float Rk3576TelemetryFilter_GetTempRight(const void *state,
                                                const volatile sensor_data_t *sensor);

/* USART1 PID 调试协议接收状态。 */
static PidDebugRxParser s_pid_debug_rx_parser;
static volatile stop_reason_t s_pending_stop_reason = STOP_REASON_NONE;
static volatile uint8_t s_pending_stop_reason_valid = 0U;

/* 发送侧滤波仅作用于当前实际上传给 RK3576 的测点，不影响控制闭环输入。 */
typedef struct
{
    TargetWindowFilter press_left;
    TargetWindowFilter temp_left;
    TargetWindowFilter temp_right;
    float press_left_value;
    float temp_left_value;
    float temp_right_value;
    uint8_t press_left_valid;
    uint8_t temp_left_valid;
    uint8_t temp_right_valid;
} Rk3576TelemetryFilterState;

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

/* 将 PID 调试串口解析出的命令转为 ctrl_cmd_t。 */
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

/*
 * 根据模式值写入脉冲阶段的开关时间。
 * 这是 MODE_SELECT 协议帧的参数落点，后续调试只需要改档位宏即可。
 */
static void apply_mode_profile(control_config_t *cfg, uint8_t mode)
{
    if (cfg == NULL)
    {
        return;
    }

    switch (mode)
    {
    case MODE_SELECT_RELAX:
        cfg->mode = MODE_SELECT_RELAX;
        cfg->pulse_on_ms = MODE_RELAX_PULSE_ON_MS;
        cfg->pulse_off_ms = MODE_RELAX_PULSE_OFF_MS;
        break;
    case MODE_SELECT_FAST:
        cfg->mode = MODE_SELECT_FAST;
        cfg->pulse_on_ms = MODE_FAST_PULSE_ON_MS;
        cfg->pulse_off_ms = MODE_FAST_PULSE_OFF_MS;
        break;
    case MODE_SELECT_STANDARD:
    default:
        cfg->mode = MODE_SELECT_STANDARD;
        cfg->pulse_on_ms = MODE_STANDARD_PULSE_ON_MS;
        cfg->pulse_off_ms = MODE_STANDARD_PULSE_OFF_MS;
        break;
    }
}

/* 给传感器任务下发采样模式，使用 overwrite 保证只保留最新意图。 */
static void send_sensor_mode(temp_acquire_mode_t mode, uint8_t suppress_rtd_fail_log)
{
    sensor_cmd_t cmd;

    if (gSensorCmdQueue == NULL)
    {
        return;
    }

    cmd.mode = mode;
    cmd.suppress_rtd_fail_log = suppress_rtd_fail_log;
    (void)xQueueOverwrite(gSensorCmdQueue, &cmd);
}

/* 上电默认治疗参数。 */
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

/*
 * 根据左右眼使能和眼罩在位状态，决定温度采样任务是否需要工作。
 * 当前实现只区分“空闲停止采样”和“双路轮询采样”。
 */
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
    if ((left_enabled != 0U) || (right_enabled != 0U))
    {
        return TEMP_ACQUIRE_MODE_DUAL_SCAN;
    }
    return TEMP_ACQUIRE_MODE_IDLE;
}

/* 单眼治疗时，允许抑制另一侧 RTD 可能产生的读数失败告警。 */
static uint8_t resolve_temperature_log_suppress(const control_config_t *cfg)
{
    const uint8_t left_enabled = (uint8_t)((cfg != NULL) &&
                                           (cfg->press_enable_L != 0U) &&
                                           (gSensorData.heaterPresentL != 0U));
    const uint8_t right_enabled = (uint8_t)((cfg != NULL) &&
                                            (cfg->press_enable_R != 0U) &&
                                            (gSensorData.heaterPresentR != 0U));

    return (uint8_t)(((left_enabled != 0U) ^ (right_enabled != 0U)) ? 1U : 0U);
}

/* 把“停机请求”重新投递给 AppTask，统一走 APP_CMD_STOP 分支收口。 */
static void request_app_stop(void)
{
    app_cmd_t cmd;

    if (gAppCommandQueue == NULL)
    {
        return;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.id = APP_CMD_STOP;
    if (xQueueSend(gAppCommandQueue, &cmd, 0U) != pdTRUE)
    {
        LOG_E("app queue full stop request");
    }
}

/* 暂存停机原因，等待 AppTask 停机时统一回传给 RK3576。 */
static void set_pending_stop_reason(stop_reason_t reason)
{
    s_pending_stop_reason = reason;
    s_pending_stop_reason_valid = 1U;
}

/* 取出并清除待处理的停机原因。 */
static uint8_t consume_pending_stop_reason(stop_reason_t *reason)
{
    uint8_t valid = s_pending_stop_reason_valid;

    if (reason != NULL)
    {
        *reason = s_pending_stop_reason;
    }

    s_pending_stop_reason = STOP_REASON_NONE;
    s_pending_stop_reason_valid = 0U;
    return valid;
}

/* 初始化单个遥测通道的目标窗口滤波器。 */
static void Rk3576TelemetryFilter_InitChannel(TargetWindowFilter *filter,
                                              uint16_t sample_count_limit,
                                              uint32_t period_ms,
                                              uint32_t now_ms)
{
    TargetWindowFilterConfig config;

    if (filter == NULL)
    {
        return;
    }

    (void)memset(&config, 0, sizeof(config));
    config.sample_count_limit = sample_count_limit;
    config.period_ms = period_ms;
    TargetWindowFilter_Init(filter, &config);
    TargetWindowFilter_Reset(filter, now_ms);
}

/*
 * 将一个采样值送入滤波器。
 * 当前策略不是平均值，而是“选择窗口内最接近目标值的样本”，
 * 目标值默认跟随上一窗口输出，用于减小上位机曲线抖动。
 */
static void Rk3576TelemetryFilter_PushChannel(TargetWindowFilter *filter,
                                              float sample,
                                              uint32_t now_ms,
                                              float *filtered_value,
                                              uint8_t *valid)
{
    float output;

    if ((filter == NULL) || (filtered_value == NULL) || (valid == NULL))
    {
        return;
    }

    if (filter->has_sample == 0U)
    {
        TargetWindowFilter_SetTarget(filter, sample);
    }

    if (TargetWindowFilter_Push(filter, sample, now_ms, NULL) == 0U)
    {
        return;
    }

    if (TargetWindowFilter_GetOutput(filter, &output) == 0U)
    {
        return;
    }

    TargetWindowFilter_SetTarget(filter, output);
    *filtered_value = output;
    *valid = 1U;
}

/* 初始化发送侧滤波状态。 */
static void Rk3576TelemetryFilter_Init(void *state_ptr, TickType_t now_tick)
{
    Rk3576TelemetryFilterState *state = (Rk3576TelemetryFilterState *)state_ptr;
    uint32_t now_ms = (uint32_t)(now_tick * portTICK_PERIOD_MS);

    if (state == NULL)
    {
        return;
    }

    (void)memset(state, 0, sizeof(*state));
    Rk3576TelemetryFilter_InitChannel(&state->press_left,
                                      RK3576_PRESS_FILTER_SAMPLES,
                                      RK3576_PRESS_FILTER_WINDOW_MS,
                                      now_ms);
    Rk3576TelemetryFilter_InitChannel(&state->temp_left,
                                      RK3576_TEMP_FILTER_SAMPLES,
                                      RK3576_TEMP_FILTER_WINDOW_MS,
                                      now_ms);
    Rk3576TelemetryFilter_InitChannel(&state->temp_right,
                                      RK3576_TEMP_FILTER_SAMPLES,
                                      RK3576_TEMP_FILTER_WINDOW_MS,
                                      now_ms);
}

/* 治疗会话边界处重置滤波器，避免跨会话串窗。 */
static void Rk3576TelemetryFilter_Reset(void *state_ptr, TickType_t now_tick)
{
    Rk3576TelemetryFilterState *state = (Rk3576TelemetryFilterState *)state_ptr;
    uint32_t now_ms = (uint32_t)(now_tick * portTICK_PERIOD_MS);

    if (state == NULL)
    {
        return;
    }

    state->press_left_value = 0.0f;
    state->temp_left_value = 0.0f;
    state->temp_right_value = 0.0f;
    state->press_left_valid = 0U;
    state->temp_left_valid = 0U;
    state->temp_right_valid = 0U;
    TargetWindowFilter_Reset(&state->press_left, now_ms);
    TargetWindowFilter_Reset(&state->temp_left, now_ms);
    TargetWindowFilter_Reset(&state->temp_right, now_ms);
}

/* 按控制周期持续向发送侧滤波器喂样本。 */
static void Rk3576TelemetryFilter_Update(void *state_ptr,
                                         const volatile sensor_data_t *sensor,
                                         TickType_t now_tick)
{
    Rk3576TelemetryFilterState *state = (Rk3576TelemetryFilterState *)state_ptr;
    uint32_t now_ms;

    if ((state == NULL) || (sensor == NULL))
    {
        return;
    }

    now_ms = (uint32_t)(now_tick * portTICK_PERIOD_MS);
    Rk3576TelemetryFilter_PushChannel(&state->press_left,
                                      sensor->pressL,
                                      now_ms,
                                      &state->press_left_value,
                                      &state->press_left_valid);
    Rk3576TelemetryFilter_PushChannel(&state->temp_left,
                                      sensor->tempL,
                                      now_ms,
                                      &state->temp_left_value,
                                      &state->temp_left_valid);
    Rk3576TelemetryFilter_PushChannel(&state->temp_right,
                                      sensor->tempR,
                                      now_ms,
                                      &state->temp_right_value,
                                      &state->temp_right_valid);
}

/* 获取左压力滤波输出，若窗口尚未成熟则回退为当前原始采样。 */
static float Rk3576TelemetryFilter_GetPressureLeft(const void *state_ptr,
                                                   const volatile sensor_data_t *sensor)
{
    const Rk3576TelemetryFilterState *state = (const Rk3576TelemetryFilterState *)state_ptr;

    if ((state != NULL) && (state->press_left_valid != 0U))
    {
        return state->press_left_value;
    }

    return (sensor != NULL) ? sensor->pressL : 0.0f;
}

/* 获取左温度滤波输出，若窗口尚未成熟则回退为当前原始采样。 */
static float Rk3576TelemetryFilter_GetTempLeft(const void *state_ptr,
                                               const volatile sensor_data_t *sensor)
{
    const Rk3576TelemetryFilterState *state = (const Rk3576TelemetryFilterState *)state_ptr;

    if ((state != NULL) && (state->temp_left_valid != 0U))
    {
        return state->temp_left_value;
    }

    return (sensor != NULL) ? sensor->tempL : 0.0f;
}

/* 获取右温度滤波输出，若窗口尚未成熟则回退为当前原始采样。 */
static float Rk3576TelemetryFilter_GetTempRight(const void *state_ptr,
                                                const volatile sensor_data_t *sensor)
{
    const Rk3576TelemetryFilterState *state = (const Rk3576TelemetryFilterState *)state_ptr;

    if ((state != NULL) && (state->temp_right_valid != 0U))
    {
        return state->temp_right_value;
    }

    return (sensor != NULL) ? sensor->tempR : 0.0f;
}

/* 协议层回调：将 UART3 解析出的业务帧投递给 AppTask。 */
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

    if (xQueueSend(gAppCommandQueue, &cmd, 0U) != pdTRUE)
    {
        LOG_E("app queue full frame=0x%04X cmd=%u",
              (unsigned int)frame->frame_id,
              (unsigned int)cmd.id);
    }
}

/* 日志串口回调：当前主要承接 PID 调试协议。 */
static void OnLogRx(void *context,
                    CommunicationChannel channel,
                    const uint8_t *data,
                    size_t length)
{
    PidDebugCommand cmd;
    size_t index;
    (void)context;

    if ((channel != COMM_CHANNEL_UART1) || (data == NULL) || (length == 0U))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        if (PidDebugProtocol_ProcessRxByte(&s_pid_debug_rx_parser, data[index], &cmd) != 0U)
        {
            send_pid_ctrl_command(&cmd);
        }
    }
}

/*
 * 应用状态机任务。
 * 职责是维护 control_config_t，并把 RK3576 命令转换为：
 * 1. ControlTask 的控制命令；
 * 2. SensorTask 的采样模式命令；
 * 3. 必要的上位机回包。
 */
static void AppTask(void *argument)
{
    app_cmd_t cmd;
    control_config_t cfg;
    uint8_t paused = 0U;

    (void)argument;
    config_defaults(&cfg);
    send_sensor_mode(TEMP_ACQUIRE_MODE_IDLE, 0U);
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
            /* 心跳只回 ACK，不改动本地状态。 */
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
            /* 左眼启停会影响加热采样模式和后续闭环所作用的治疗侧。 */
            cfg.press_enable_L = (cmd.v.u8 != 0U) ? 1U : 0U;
            if (cfg.running != 0U)
            {
                send_sensor_mode(resolve_temperature_mode(&cfg),
                                 resolve_temperature_log_suppress(&cfg));
            }
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_RIGHT_ENABLE:
            /* 右眼启停逻辑与左眼对称。 */
            cfg.press_enable_R = (cmd.v.u8 != 0U) ? 1U : 0U;
            if (cfg.running != 0U)
            {
                send_sensor_mode(resolve_temperature_mode(&cfg),
                                 resolve_temperature_log_suppress(&cfg));
            }
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_LEFT_HEATER_FUSE_BLOW:
            //EyeShieldStatus_RequestFuseBlow(1U, 0U);
            break;
        case APP_CMD_RIGHT_HEATER_FUSE_BLOW:
            //EyeShieldStatus_RequestFuseBlow(0U, 1U);
            break;
        case APP_CMD_SET_TREATMENT_TIME:
            cfg.treatment_minutes = (cmd.v.u16 == 0U) ? 1U : cmd.v.u16;
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_MODE_SELECT:
            /* 模式切换只改参数，不隐式启动治疗。 */
            apply_mode_profile(&cfg, cmd.v.u8);
            send_ctrl_command(CTRL_CMD_UPDATE_CFG, &cfg);
            break;
        case APP_CMD_START:
            /* 若上位机未显式指定左右眼，则默认双眼治疗。 */
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
            send_sensor_mode(resolve_temperature_mode(&cfg),
                             resolve_temperature_log_suppress(&cfg));
            send_ctrl_command(CTRL_CMD_START, &cfg);
            break;
        case APP_CMD_STOP:
        {
            stop_reason_t stop_reason = STOP_REASON_NONE;
            uint8_t stop_reason_valid = consume_pending_stop_reason(&stop_reason);

            paused = 0U;
            cfg.running = 0U;
            gTreatmentRunning = 0U;
            LOG_I("app stop");
            send_sensor_mode(TEMP_ACQUIRE_MODE_IDLE, 0U);
            if (stop_reason_valid == 0U)
            {
                stop_reason = STOP_REASON_MANUAL;
            }
            enqueue_tx_u8(PROTOCOL_ID_U8_STOP_REASON, (uint8_t)stop_reason);
            send_ctrl_command(CTRL_CMD_STOP, &cfg);
            break;
        }
        case APP_CMD_PAUSE_RESUME:
            /* 协议约定：0=暂停，非 0=恢复。 */
            if (cmd.v.u8 == 0U)
            {
                paused = 1U;
                gTreatmentRunning = 0U;
                LOG_I("app pause");
                send_sensor_mode(TEMP_ACQUIRE_MODE_IDLE, 0U);
                send_ctrl_command(CTRL_CMD_PAUSE, &cfg);
            }
            else
            {
                paused = 0U;
                gTreatmentRunning = (cfg.running != 0U) ? 1U : 0U;
                LOG_I("app resume");
                send_sensor_mode(resolve_temperature_mode(&cfg),
                                 resolve_temperature_log_suppress(&cfg));
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

/* 周期轮询通信模块，驱动 DMA/IDLE 串口收包状态机前进。 */
static void CommRxTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        Communication_PollRx();
        vTaskDelay(pdMS_TO_TICKS(COMM_RX_PERIOD_MS));
    }
}

/* 统一发送任务：把抽象 tx_frame_t 转换成真实协议帧发往 UART3。 */
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

/*
 * 传感器任务：
 * - 温度：通过 ADS1248 轮询/中断配合更新；
 * - 压力：固定周期轮询更新；
 * 最终都写入共享的 gSensorData。
 */
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
            TreatmentTemperatureSensor_SetMode(&temperature_sensor,
                                               sensor_cmd.mode,
                                               sensor_cmd.suppress_rtd_fail_log);
        }

        now = xTaskGetTickCount();
        TreatmentTemperatureSensor_Poll(&temperature_sensor, &gSensorData);
        TreatmentPressureSensor_Poll(&pressure_sensor, now, &gSensorData);
        gSensorData.tick = now;
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/*
 * 控制任务是治疗主闭环：
 * - 接收 ctrl_cmd_t；
 * - 维护眼罩在位/保险丝保护；
 * - 执行压力和加热控制；
 * - 发送温度、压力及 PID 调试信息。
 */
static void ControlTask(void *argument)
{
    TreatmentAppController controller;
    TreatmentAppRuntime runtime;
    Rk3576TelemetryFilterState telemetry_filter;
    ctrl_cmd_t cmd;
    TickType_t now;
    TickType_t next_press_tx;
    TickType_t next_temp_tx;
    TickType_t next_eye_shield_status;
    TickType_t next_control_log;
    TickType_t next_pid_debug_tx;
    uint8_t telemetry_filter_active = 0U;
    uint8_t telemetry_tx_enabled = 0U;

    (void)argument;
    TreatmentAppController_Init(&controller);
    EyeShieldStatus_Init();
    now = xTaskGetTickCount();
    Rk3576TelemetryFilter_Init(&telemetry_filter, now);
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

        /* 眼罩状态检查属于控制前置安全条件。 */
        EyeShieldStatus_Service();
        if ((int32_t)(now - next_eye_shield_status) >= 0)
        {
            stop_reason_t eye_shield_stop_reason = STOP_REASON_EYE_SHIELD_OFFLINE;

            next_eye_shield_status = now + pdMS_TO_TICKS(EYE_SHIELD_STATUS_MS);
            if (EyeShieldStatus_Process(&controller.cfg, &eye_shield_stop_reason) != 0U)
            {
                LOG_E("eye shield stop during treatment, reason=%u", (unsigned int)eye_shield_stop_reason);
                set_pending_stop_reason(eye_shield_stop_reason);
                send_ctrl_command(CTRL_CMD_STOP, NULL);
                gTreatmentRunning = 0U;
                request_app_stop();
            }
        }

        TreatmentAppController_Run(&controller,&gSensorData,now,(float)CONTROL_PERIOD_MS / 1000.0f, &runtime);
        telemetry_tx_enabled = (uint8_t)((runtime.session_active != 0U) &&
                                         (controller.paused == 0U));

#if RK3576_TELEMETRY_FILTER_ENABLE
        /* 发送侧滤波只影响上位机曲线，不参与控制闭环。 */
        if (telemetry_tx_enabled != 0U)
        {
            if (telemetry_filter_active == 0U)
            {
                Rk3576TelemetryFilter_Reset(&telemetry_filter, now);
                telemetry_filter_active = 1U;
            }
            Rk3576TelemetryFilter_Update(&telemetry_filter, &gSensorData, now);
        }
        else if (telemetry_filter_active != 0U)
        {
            Rk3576TelemetryFilter_Reset(&telemetry_filter, now);
            telemetry_filter_active = 0U;
        }
#else
        (void)telemetry_filter;
        (void)telemetry_filter_active;
#endif
        if (telemetry_tx_enabled == 0U)
        {
            next_press_tx = now + pdMS_TO_TICKS(PRESS_TELEMETRY_MS);
            next_temp_tx = now + pdMS_TO_TICKS(TEMP_TELEMETRY_MS);
        }

        if ((controller.cfg.running != 0U) && (controller.cfg.treatment_minutes > 0U))
        {
            /* 达到总治疗时长后，通过统一停机链路结束治疗。 */
            uint32_t elapsed_ms =
                (uint32_t)((now - controller.wave_anchor_tick) * portTICK_PERIOD_MS);
            uint32_t duration_ms = (uint32_t)controller.cfg.treatment_minutes * 60U * 1000U;

            if (elapsed_ms >= duration_ms)
            {
                set_pending_stop_reason(STOP_REASON_NONE);
                send_ctrl_command(CTRL_CMD_STOP, NULL);
                gTreatmentRunning = 0U;
                request_app_stop();
                vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
                continue;
            }
        }

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

        if ((telemetry_tx_enabled != 0U) &&
            ((int32_t)(now - next_press_tx) >= 0))
        {
            next_press_tx = now + pdMS_TO_TICKS(PRESS_TELEMETRY_MS);
#if RK3576_TELEMETRY_FILTER_ENABLE
            /* 当前协议主链路只发左压力值，右压力帧 ID 仍保留作后续扩展。 */
            enqueue_tx_f32(PROTOCOL_ID_F32_LEFT_PRESSURE_VALUE,
                           Rk3576TelemetryFilter_GetPressureLeft(&telemetry_filter, &gSensorData));
#else
            enqueue_tx_f32(PROTOCOL_ID_F32_LEFT_PRESSURE_VALUE, gSensorData.pressL);
#endif
        }

        if ((telemetry_tx_enabled != 0U) &&
            ((int32_t)(now - next_temp_tx) >= 0))
        {
            next_temp_tx = now + pdMS_TO_TICKS(TEMP_TELEMETRY_MS);
#if RK3576_TELEMETRY_FILTER_ENABLE
            enqueue_tx_f32(PROTOCOL_ID_F32_LEFT_TEMP_VALUE,
                           Rk3576TelemetryFilter_GetTempLeft(&telemetry_filter, &gSensorData));
            enqueue_tx_f32(PROTOCOL_ID_F32_RIGHT_TEMP_VALUE,
                           Rk3576TelemetryFilter_GetTempRight(&telemetry_filter, &gSensorData));
#else
            enqueue_tx_f32(PROTOCOL_ID_F32_LEFT_TEMP_VALUE, gSensorData.tempL);
            enqueue_tx_f32(PROTOCOL_ID_F32_RIGHT_TEMP_VALUE, gSensorData.tempR);
#endif
            //enqueue_tx_u8(PROTOCOL_ID_U8_MODE_CURVES, runtime.phase_char);
        }

        if ((controller.pid_debug_stream_enabled != 0U) &&
            ((int32_t)(now - next_pid_debug_tx) >= 0))
        {
            /* PID 调试数据走日志串口格式，不占用 RK3576 业务协议。 */
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

/* 独立安全任务：检测超温/超压并触发应用层停机。 */
static void SafetyTask(void *argument)
{
    uint8_t fault_active = 0U;
    (void)argument;

    for (;;)
    {
        stop_reason_t stop_reason = STOP_REASON_NONE;
        uint8_t heat_otp_fault_left = 0U;
        uint8_t heat_otp_fault_right = 0U;
        uint8_t heat_otp_level_left = 0U;
        uint8_t heat_otp_level_right = 0U;
        uint8_t over_temp_fault;
        uint8_t over_pressure_fault;
        uint8_t heat_otp_fault;
        uint8_t fault =
            0U;

        heat_otp_level_left =
            (uint8_t)((HAL_GPIO_ReadPin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin) == GPIO_PIN_SET) ? 1U : 0U);
        heat_otp_level_right =
            (uint8_t)((HAL_GPIO_ReadPin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin) == GPIO_PIN_SET) ? 1U : 0U);
        TreatmentHeatingControl_GetOtpFaultFlags(&heat_otp_fault_left, &heat_otp_fault_right);
        over_temp_fault =
            (uint8_t)((gSensorData.tempL > TEMP_MAX_C) ||
                      (gSensorData.tempR > TEMP_MAX_C));
        over_pressure_fault =
            (uint8_t)((gSensorData.pressL > PRESS_MAX_KPA) ||
                      (gSensorData.pressR > PRESS_MAX_KPA));
        heat_otp_fault =
            (uint8_t)((heat_otp_fault_left != 0U) ||
                      (heat_otp_fault_right != 0U));
        fault = (uint8_t)((over_temp_fault != 0U) ||
                          (over_pressure_fault != 0U) ||
                          (heat_otp_fault != 0U));

        if ((fault != 0U) && (fault_active == 0U))
        {
            LOG_E("safety fault detect tempL=%d.%02d tempR=%d.%02d pressL=%d.%02d pressR=%d.%02d otpL_level=%u otpR_level=%u otpL_fault=%u otpR_fault=%u",
                  (int)gSensorData.tempL,
                  (int)((gSensorData.tempL - (float)((int)gSensorData.tempL)) * 100.0f),
                  (int)gSensorData.tempR,
                  (int)((gSensorData.tempR - (float)((int)gSensorData.tempR)) * 100.0f),
                  (int)gSensorData.pressL,
                  (int)((gSensorData.pressL - (float)((int)gSensorData.pressL)) * 100.0f),
                  (int)gSensorData.pressR,
                  (int)((gSensorData.pressR - (float)((int)gSensorData.pressR)) * 100.0f),
                  heat_otp_level_left,
                  heat_otp_level_right,
                  heat_otp_fault_left,
                  heat_otp_fault_right);
            if ((over_temp_fault != 0U) || (heat_otp_fault != 0U))
            {
                stop_reason = STOP_REASON_OVER_TEMP;
            }
            else if (over_pressure_fault != 0U)
            {
                stop_reason = STOP_REASON_OVER_PRESSURE;
            }
            set_pending_stop_reason(stop_reason);
            send_ctrl_command(CTRL_CMD_STOP, NULL);
            gTreatmentRunning = 0U;
            request_app_stop();
            fault_active = 1U;
        }
        else if (fault == 0U)
        {
            fault_active = 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}

/* 注册通信回调并创建全部应用任务。 */
void AppTasks_Init(void)
{
    CommunicationCallbacks callbacks;
    (void)memset(&callbacks, 0, sizeof(callbacks));
    PidDebugProtocol_InitRxParser(&s_pid_debug_rx_parser);
    callbacks.on_protocol_frame = OnProtocolFrame;
    callbacks.on_log_rx = OnLogRx;
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
