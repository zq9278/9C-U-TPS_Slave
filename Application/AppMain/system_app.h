#ifndef SYSTEM_APP_H
#define SYSTEM_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

// ---------------- Sensor data ----------------
typedef struct {
    float tempL;     // 左眼温度(摄氏度)
    float tempR;     // 右眼温度(摄氏度)
    float pressL;    // 左眼压力(kPa)
    float pressR;    // 右眼压力(kPa)
    uint32_t tick;   // 采样时间戳(FreeRTOS tick)
} sensor_data_t;

extern volatile sensor_data_t gSensorData;

// ---------------- App state ----------------
typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_READY,
    APP_STATE_RUN_MODE1,
    APP_STATE_RUN_MODE2,
    APP_STATE_ALARM,
} app_state_t;

extern volatile app_state_t gAppState;

// ---------------- Commands from Comm -> App ----------------
typedef enum {
    APP_CMD_NONE = 0,
    APP_CMD_START,
    APP_CMD_STOP,
    APP_CMD_MODE_SELECT,       // u8: 1/2
    APP_CMD_SET_TEMP_LEFT,     // float °C
    APP_CMD_SET_TEMP_RIGHT,    // float °C
    APP_CMD_SET_PRESSURE_KPA,  // float kPa (generic)
    APP_CMD_SET_MODE1_PARAM,   // structure index-based updates
    APP_CMD_SET_MODE2_PARAM,
    APP_CMD_READ_PARAM,
    APP_CMD_SAVE_PARAM,
    APP_CMD_SET_LEFT_PRESSURE_ENABLE,   // u8 0/1
    APP_CMD_SET_RIGHT_PRESSURE_ENABLE,  // u8 0/1
} app_cmd_id_t;

typedef struct {
    app_cmd_id_t id;   // 命令ID
    uint16_t key;      // 子字段键值(用于同一命令区分不同参数)
    union {
        float    f32;  // 浮点数值
        uint32_t u32;  // 32位无符号数
        uint16_t u16;  // 16位无符号数
        uint8_t  u8;   // 8位无符号数
    } v;               // 数据载荷
} app_cmd_t;

// ---------------- Control config (App -> Control) ----------------
typedef struct {
    float temp_target;      // 加热温度目标(摄氏度)
    float press_target_max; // 压力目标(kPa)
    float t1_rise_s;        // 缓慢上升阶段(秒)
    float t2_hold_s;        // 恒压阶段(秒)
    float t3_pulse_s;       // 脉动阶段总时长(秒)
    float pulse_on_ms;      // 脉动ON时间(毫秒)
    float pulse_off_ms;     // 脉动OFF时间(毫秒)
    uint8_t mode;           // 模式号(固定1)
    uint8_t running;        // 是否运行(1启动/0停止)
    uint8_t squeeze_mode;   // 挤压模式(0同步/1交替/2同步)
    uint8_t press_enable_L; // 左侧压力开关(0关/1开)
    uint8_t press_enable_R; // 右侧压力开关(0关/1开)
} control_config_t;

typedef enum {
    CTRL_CMD_NONE = 0,
    CTRL_CMD_START,
    CTRL_CMD_STOP,
    CTRL_CMD_UPDATE_CFG,
} ctrl_cmd_id_t;

typedef struct {
    ctrl_cmd_id_t id;     // 控制命令ID
    control_config_t cfg; // 控制参数载荷
} ctrl_cmd_t;

// ---------------- Storage commands ----------------
typedef enum {
    STORAGE_CMD_LOAD_ALL = 0,
    STORAGE_CMD_SAVE_PARAM,
} storage_cmd_t;

// ---------------- TX frames from other tasks to Comm ----------------
typedef enum {
    TX_DATA_UINT8 = 1,
    TX_DATA_FLOAT = 2,
    TX_DATA_TEXT  = 3,
    TX_DATA_U16   = 4,
    TX_DATA_U32   = 5,
} tx_data_type_t;

typedef struct {
    uint16_t frame_id;   // 帧ID
    tx_data_type_t type; // 数据类型
    union {
        float f32;       // 浮点数据
        uint32_t u32;    // 32位无符号
        uint16_t u16;    // 16位无符号
        uint8_t u8;      // 8位无符号
        char text[32];   // 文本
    } v;                 // 载荷
} tx_frame_t;

// ---------------- Queues ----------------
extern QueueHandle_t gCmdQueue;       // Comm -> App
extern QueueHandle_t gCtrlCmdQueue;   // App -> Control
extern QueueHandle_t gTxQueue;        // any -> Comm
extern QueueHandle_t gStorageQueue;   // App -> Storage
extern QueueHandle_t gSafetyQueue;    // Safety -> App

// ---------------- Task entry points ----------------
void CommTask(void *argument);
void AppTask(void *argument);
void SensorTask(void *argument);
void ControlTask(void *argument);
void StorageTask(void *argument);
void IoTask(void *argument);
void SafetyTask(void *argument);

#endif // SYSTEM_APP_H
