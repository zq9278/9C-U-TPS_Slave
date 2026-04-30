#ifndef SYSTEM_APP_H
#define SYSTEM_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- Sensor data ----------------
typedef struct {
    float tempL;     // left eye temperature
    float tempR;     // right eye temperature
    float pressL;    // left eye pressure
    float pressR;    // right eye pressure
    uint8_t heaterPresentL; // 1 = present, 0 = absent
    uint8_t heaterPresentR; // 1 = present, 0 = absent
    uint8_t heaterFuseL;    // 1 = normal, 0 = blown
    uint8_t heaterFuseR;    // 1 = normal, 0 = blown
    uint32_t tick;   // FreeRTOS sampling tick
} sensor_data_t;

extern volatile sensor_data_t gSensorData;
extern volatile uint8_t gTreatmentRunning;

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
    APP_CMD_PAUSE_RESUME,
    APP_CMD_MODE_SELECT,
    APP_CMD_SET_TEMP,
    APP_CMD_SET_PRESSURE_KPA,
    APP_CMD_SET_TREATMENT_TIME,
    APP_CMD_LEFT_ENABLE,
    APP_CMD_RIGHT_ENABLE,
    APP_CMD_READ_PARAM,
    APP_CMD_SAVE_PARAM,
} app_cmd_id_t;

typedef struct {
    app_cmd_id_t id;
    uint16_t key;
    union {
        float    f32;
        uint32_t u32;
        uint16_t u16;
        uint8_t  u8;
    } v;
} app_cmd_t;

typedef enum {
    APP_EVT_NONE = 0,
    APP_EVT_HOST_COMMAND,
    APP_EVT_SAFETY_FAULT,
} app_event_id_t;

typedef struct {
    app_event_id_t id;
    union {
        app_cmd_t host_cmd;
        uint8_t safety_fault;
    } v;
} app_event_t;

// ---------------- Control config (App -> Control) ----------------
typedef struct {
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

typedef enum {
    CTRL_CMD_NONE = 0,
    CTRL_CMD_START,
    CTRL_CMD_STOP,
    CTRL_CMD_PAUSE,
    CTRL_CMD_RESUME,
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
extern QueueHandle_t gAppEventQueue;
extern QueueHandle_t gCtrlCmdQueue;
extern QueueHandle_t gTxQueue;
extern QueueHandle_t gStorageQueue;

// ---------------- Task entry points ----------------
void CommTask(void *argument);
void DebugLogTask(void *argument);
void AppTask(void *argument);
void SensorTask(void *argument);
void ControlTask(void *argument);
void StorageTask(void *argument);
void IoTask(void *argument);
void SafetyTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_APP_H
