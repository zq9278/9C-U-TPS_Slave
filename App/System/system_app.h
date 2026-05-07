#ifndef APP_SYSTEM_SYSTEM_APP_H
#define APP_SYSTEM_SYSTEM_APP_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float tempL;
    float tempR;
    float pressL;
    float pressR;
    uint8_t heaterPresentL;
    uint8_t heaterPresentR;
    uint8_t heaterFuseL;
    uint8_t heaterFuseR;
    uint32_t tick;
} sensor_data_t;

typedef struct
{
    float temp_target;
    float press_target_max;
    float t1_rise_s;
    float t2_hold_s;
    float t3_pulse_s;
    float pulse_on_ms;
    float pulse_off_ms;
    uint8_t mode;
    uint8_t running;
    uint8_t squeeze_mode;
    uint8_t press_enable_L;
    uint8_t press_enable_R;
    uint16_t treatment_minutes;
} control_config_t;

typedef enum
{
    PID_DEBUG_TARGET_PRESS_RISE = 0,
    PID_DEBUG_TARGET_PRESS_HOLD,
    PID_DEBUG_TARGET_PRESS_PULSE,
    PID_DEBUG_TARGET_HEAT_LEFT,
    PID_DEBUG_TARGET_HEAT_RIGHT
} pid_debug_target_t;

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

typedef enum
{
    TEMP_ACQUIRE_MODE_IDLE = 0,
    TEMP_ACQUIRE_MODE_DUAL_SCAN,
} temp_acquire_mode_t;

typedef struct
{
    temp_acquire_mode_t mode;
    uint8_t suppress_rtd_fail_log;
} sensor_cmd_t;

typedef enum
{
    STOP_REASON_NONE = 0,
    STOP_REASON_MANUAL = 1,
    STOP_REASON_EYE_SHIELD_OFFLINE = 2,
    STOP_REASON_OVER_TEMP = 3,
    STOP_REASON_OVER_PRESSURE = 4
} stop_reason_t;

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

typedef enum
{
    TX_DATA_UINT8 = 1,
    TX_DATA_FLOAT = 2,
    TX_DATA_TEXT = 3,
    TX_DATA_U16 = 4,
    TX_DATA_U32 = 5,
} tx_data_type_t;

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

extern volatile sensor_data_t gSensorData;
extern volatile uint8_t gTreatmentRunning;

extern QueueHandle_t gAppCommandQueue;
extern QueueHandle_t gCtrlCmdQueue;
extern QueueHandle_t gSensorCmdQueue;
extern QueueHandle_t gTxQueue;

#ifdef __cplusplus
}
#endif

#endif
