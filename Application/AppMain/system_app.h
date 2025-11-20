#ifndef SYSTEM_APP_H
#define SYSTEM_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

// ---------------- Sensor data ----------------
typedef struct {
    float tempL;     // °C
    float tempR;     // °C
    float pressL;    // kPa
    float pressR;    // kPa
    uint32_t tick;   // FreeRTOS tick at sampling
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
    APP_CMD_SET_USE_COMMON_TEMP, // u8 0/1
    APP_CMD_SET_PRESSURE_KPA,  // float kPa (generic)
    APP_CMD_SET_MODE1_PARAM,   // structure index-based updates
    APP_CMD_SET_MODE2_PARAM,
    APP_CMD_READ_PARAM,
    APP_CMD_SAVE_PARAM,
    APP_CMD_SET_LEFT_PRESSURE_ENABLE,   // u8 0/1
    APP_CMD_SET_RIGHT_PRESSURE_ENABLE,  // u8 0/1
} app_cmd_id_t;

typedef struct {
    app_cmd_id_t id;
    uint16_t key;      // sub-field key when needed
    union {
        float    f32;
        uint32_t u32;
        uint16_t u16;
        uint8_t  u8;
    } v;
} app_cmd_t;

// ---------------- Control config (App -> Control) ----------------
typedef struct {
    float temp_target;      // °C, same for L/R if use_common
    float press_target_max; // kPa
    float t1_rise_s;        // s
    float t2_hold_s;        // s
    float t3_pulse_s;       // s
    float pulse_on_ms;      // ms
    float pulse_off_ms;     // ms
    uint8_t mode;           // 1 or 2
    uint8_t running;        // 0/1
    uint8_t squeeze_mode;   // 0 normal, 1 alternate, 2 sync
    uint8_t press_enable_L; // 0/1 独立左侧压力开�?    
    uint8_t press_enable_R; // 0/1 独立右侧压力开�?
    } control_config_t;

typedef enum {
    CTRL_CMD_NONE = 0,
    CTRL_CMD_START,
    CTRL_CMD_STOP,
    CTRL_CMD_UPDATE_CFG,
} ctrl_cmd_id_t;

typedef struct {
    ctrl_cmd_id_t id;
    control_config_t cfg;
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
    uint16_t frame_id;
    tx_data_type_t type;
    union {
        float f32;
        uint32_t u32;
        uint16_t u16;
        uint8_t u8;
        char text[32];
    } v;
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
