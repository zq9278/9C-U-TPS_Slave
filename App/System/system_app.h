#ifndef APP_SYSTEM_SYSTEM_APP_H
#define APP_SYSTEM_SYSTEM_APP_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 传感器共享快照。
 *
 * 由 SensorTask 周期性刷新，其他任务只读。
 * 温度/压力值均为当前最近一次有效滤波结果。
 */
typedef struct
{
    float tempL;              /* 左眼温度，单位摄氏度。 */
    float tempR;              /* 右眼温度，单位摄氏度。 */
    float pressL;             /* 左眼压力，单位 kPa。 */
    float pressR;             /* 右眼压力，单位 kPa。 */
    uint8_t heaterPresentL;   /* 左眼眼罩在位状态，1 表示检测到。 */
    uint8_t heaterPresentR;   /* 右眼眼罩在位状态，1 表示检测到。 */
    uint8_t heaterFuseL;      /* 左眼保险丝状态，1 表示完好。 */
    uint8_t heaterFuseR;      /* 右眼保险丝状态，1 表示完好。 */
    uint32_t tick;            /* 本次采样对应的系统 Tick。 */
} sensor_data_t;

/**
 * @brief 治疗控制配置。
 *
 * 该结构代表“当前希望系统如何治疗”，由 AppTask 汇总生成，
 * 再通过 ctrl_cmd_t 下发给 ControlTask 内部控制器。
 */
typedef struct
{
    float temp_target;        /* 温度目标值，单位摄氏度。 */
    float press_target_max;   /* 压力峰值目标，单位 kPa。 */
    float t1_rise_s;          /* 升压阶段时长，单位秒。 */
    float t2_hold_s;          /* 保压阶段时长，单位秒。 */
    float t3_pulse_s;         /* 脉冲阶段总时长，单位秒。 */
    float pulse_on_ms;        /* 脉冲阶段单次导通时长，单位毫秒。 */
    float pulse_off_ms;       /* 脉冲阶段单次泄压时长，单位毫秒。 */
    uint8_t mode;             /* RK3576 下发的治疗档位/模式值。 */
    uint8_t running;          /* 逻辑运行标志。 */
    uint8_t squeeze_mode;     /* 预留的按压模式参数，当前主流程未深度使用。 */
    uint8_t press_enable_L;   /* 左眼治疗使能。 */
    uint8_t press_enable_R;   /* 右眼治疗使能。 */
    uint16_t treatment_minutes; /* 治疗总时长，单位分钟。 */
} control_config_t;

/* PID 调试目标枚举，既用于调试协议，也用于控制模块参数选择。 */
typedef enum
{
    PID_DEBUG_TARGET_PRESS_RISE = 0,
    PID_DEBUG_TARGET_PRESS_HOLD,
    PID_DEBUG_TARGET_PRESS_PULSE,
    PID_DEBUG_TARGET_HEAT_LEFT,
    PID_DEBUG_TARGET_HEAT_RIGHT
} pid_debug_target_t;

/* AppTask 向 ControlTask 发送的控制命令种类。 */
typedef enum
{
    CTRL_CMD_NONE = 0,
    CTRL_CMD_START,
    CTRL_CMD_STOP,
    CTRL_CMD_PAUSE,
    CTRL_CMD_RESUME,
    CTRL_CMD_UPDATE_CFG,
    CTRL_CMD_PID_SET_GAINS,
    CTRL_CMD_PID_STREAM_ENABLE,
} ctrl_cmd_id_t;

/**
 * @brief ControlTask 命令包。
 *
 * 根据 id 的不同，结构中仅部分字段有效：
 * - START/UPDATE_CFG 使用 cfg；
 * - PID_SET_GAINS 使用 pid_target/kp/ki/kd；
 * - PID_STREAM_ENABLE 使用 enabled。
 */
typedef struct
{
    ctrl_cmd_id_t id;
    control_config_t cfg;
    pid_debug_target_t pid_target;
    float kp;
    float ki;
    float kd;
    uint8_t enabled;
} ctrl_cmd_t;

/* 温度采样策略。当前主要区分“空闲停止采样”和“双通道轮询采样”。 */
typedef enum
{
    TEMP_ACQUIRE_MODE_IDLE = 0,
    TEMP_ACQUIRE_MODE_DUAL_SCAN,
    TEMP_ACQUIRE_MODE_SINGLE_LEFT,
    TEMP_ACQUIRE_MODE_SINGLE_RIGHT,
} temp_acquire_mode_t;

/* AppTask 下发给 SensorTask 的采样控制命令。 */
typedef struct
{
    temp_acquire_mode_t mode;
    uint8_t suppress_rtd_fail_log;
} sensor_cmd_t;

/* 停机原因上报码。 */
typedef enum
{
    STOP_REASON_NONE = 0,
    STOP_REASON_MANUAL = 1,
    STOP_REASON_EYE_SHIELD_OFFLINE = 2,
    STOP_REASON_OVER_TEMP = 3,
    STOP_REASON_OVER_PRESSURE = 4,
    STOP_REASON_LEFT_HEATER_PROTECTOR_FAULT = 5,
    STOP_REASON_RIGHT_HEATER_PROTECTOR_FAULT = 6
} stop_reason_t;

/* RK3576 协议帧转换后的应用级命令。 */
typedef enum
{
    APP_CMD_NONE = 0,
    APP_CMD_HEARTBEAT,
    APP_CMD_START,
    APP_CMD_STOP,
    APP_CMD_PAUSE_RESUME,
    APP_CMD_MODE_SELECT,
    APP_CMD_SET_TEMP,
    APP_CMD_SET_PRESSURE_KPA,
    APP_CMD_SET_TREATMENT_TIME,
    APP_CMD_LEFT_ENABLE,
    APP_CMD_RIGHT_ENABLE,
    APP_CMD_LEFT_HEATER_FUSE_BLOW,
    APP_CMD_RIGHT_HEATER_FUSE_BLOW,
    APP_CMD_SAVE_PARAM,
} app_cmd_id_t;

/**
 * @brief AppTask 命令包。
 *
 * 由协议层把 RK3576 帧转换成统一命令格式后压入 gAppCommandQueue。
 */
typedef struct
{
    app_cmd_id_t id;
    union
    {
        float f32;
        uint32_t u32;
        uint16_t u16;
        uint8_t u8;
    } v;
} app_cmd_t;

/* 统一发送队列支持的数据类型，最终映射到 RK3576 协议的数据类型字段。 */
typedef enum
{
    TX_DATA_UINT8 = 1,
    TX_DATA_FLOAT = 2,
    TX_DATA_TEXT = 3,
    TX_DATA_U16 = 4,
    TX_DATA_U32 = 5,
} tx_data_type_t;

/**
 * @brief 上行发送帧描述。
 *
 * 各任务只负责填充 frame_id、type 和 value，
 * 实际串口打包发送由 CommTxTask/Communication 模块统一完成。
 */
typedef struct
{
    uint16_t frame_id;
    tx_data_type_t type;
    union
    {
        float f32;
        uint32_t u32;
        uint16_t u16;
        uint8_t u8;
        char text[32];
    } v;
} tx_frame_t;

/* 共享运行态变量。 */
extern volatile sensor_data_t gSensorData;
extern volatile uint8_t gTreatmentRunning;

/* 全局队列句柄。 */
extern QueueHandle_t gAppCommandQueue;
extern QueueHandle_t gCtrlCmdQueue;
extern QueueHandle_t gSensorCmdQueue;
extern QueueHandle_t gTxQueue;

#ifdef __cplusplus
}
#endif

#endif
