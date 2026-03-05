/*
 * UART protocol dispatch: parse frames and push app_cmd_t
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "Uart_Communicate.h"
#include "AppMain/system_app.h"
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "LOG.h"

/* Pressure compensation applied right after UART RX (host setpoint -> internal setpoint). */
#define PRESSURE_SET_COMP_ADD    (0.0f)//压力补偿
#define PRESSURE_SET_COMP_MIN    (0.0f)
#define PRESSURE_SET_COMP_MAX    (400.0f)

static float compensate_pressure_set(float raw_set)
{
    float v = raw_set + PRESSURE_SET_COMP_ADD;
    if (v < PRESSURE_SET_COMP_MIN) v = PRESSURE_SET_COMP_MIN;
    if (v > PRESSURE_SET_COMP_MAX) v = PRESSURE_SET_COMP_MAX;
    return v;
}

// Data parse helpers
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len)
{
    (void)data_ptr; (void)data_len;
}

uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    uint8_t v = 0;
    if (data_len >= 1) memcpy(&v, data_ptr, 1);
    return v;
}

float handle_float_data(const uint8_t *data_ptr, uint16_t data_len)
{
    float v = 0.0f;
    if (data_len >= sizeof(float)) memcpy(&v, data_ptr, sizeof(float));
    return v;
}

uint16_t handle_uint16_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    uint16_t v = 0;
    if (data_len >= sizeof(uint16_t)) memcpy(&v, data_ptr, sizeof(uint16_t));
    return v;
}

// TX helper to wrap a single uint8
static inline void tx_u8(uint16_t id, uint8_t v)
{
    tx_frame_t tx = {0};
    tx.type = TX_DATA_UINT8;
    tx.frame_id = id;
    tx.v.u8 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static inline void push_cmd_u8(app_cmd_id_t id, uint8_t v)
{
    app_cmd_t c = {0};
    c.id = id;
    c.v.u8 = v;
    (void)xQueueSend(gCmdQueue, &c, 0);
}

static inline void push_cmd_f32(app_cmd_id_t id, float v)
{
    app_cmd_t c = {0};
    c.id = id;
    c.v.f32 = v;
    (void)xQueueSend(gCmdQueue, &c, 0);
}


static inline void push_cmd_u16(app_cmd_id_t id, uint16_t v)
{
    app_cmd_t c = {0};
    c.id = id;
    c.v.u16 = v;
    (void)xQueueSend(gCmdQueue, &c, 0);
}
// Frame dispatch
// Parse host frames and convert to app commands:
// - heartbeat -> ACK
// - pressure/temperature -> APP_CMD_SET_PRESSURE_KPA / APP_CMD_SET_TEMP
// - left/right enable -> APP_CMD_LEFT_ENABLE / APP_CMD_RIGHT_ENABLE
// - mode/start/stop/save/time -> corresponding app_cmd_t
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        case U8_HEARTBEAT_REQ:             // Heartbeat request -> respond
            tx_u8(U8_HEARTBEAT_ACK, 1);
            break;

        case F32_PRESSURE_SET_KPA:         // Pressure set
        {
            float raw_set = handle_float_data(data_ptr, data_len);
            float comp_set = compensate_pressure_set(raw_set);
            push_cmd_f32(APP_CMD_SET_PRESSURE_KPA, comp_set);
            break;
        }

        case F32_LEFT_TEMP_SET_C:          // Temperature set
            push_cmd_f32(APP_CMD_SET_TEMP, handle_float_data(data_ptr, data_len));
            break;

        case U8_LEFT_EYE_ENABLE:           // Left eye enable
            push_cmd_u8(APP_CMD_LEFT_ENABLE, handle_uint8_t_data(data_ptr, data_len));
            break;

        case U8_RIGHT_EYE_ENABLE:          // Right eye enable
            push_cmd_u8(APP_CMD_RIGHT_ENABLE, handle_uint8_t_data(data_ptr, data_len));
            break;

        case U8_LEFT_HEATER_FUSE_BLOW_CMD: // Force blow left fuse
            // HAL_GPIO_WritePin(Heat1_Fuse_Blown_GPIO_Port, Heat1_Fuse_Blown_Pin, GPIO_PIN_SET);
            // HAL_GPIO_WritePin(Heat2_Fuse_Blown_GPIO_Port, Heat2_Fuse_Blown_Pin, GPIO_PIN_SET);
            // HAL_Delay(200);
            // HAL_GPIO_WritePin(Heat1_Fuse_Blown_GPIO_Port, Heat1_Fuse_Blown_Pin, GPIO_PIN_RESET);
            // HAL_GPIO_WritePin(Heat2_Fuse_Blown_GPIO_Port, Heat2_Fuse_Blown_Pin, GPIO_PIN_RESET);
            break;

        case U8_RIGHT_HEATER_FUSE_BLOW_CMD: // Force blow right fuse
            HAL_GPIO_WritePin(Heat2_Fuse_Blown_GPIO_Port, Heat2_Fuse_Blown_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Heat1_Fuse_Blown_GPIO_Port, Heat1_Fuse_Blown_Pin, GPIO_PIN_SET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(Heat2_Fuse_Blown_GPIO_Port, Heat2_Fuse_Blown_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Heat1_Fuse_Blown_GPIO_Port, Heat1_Fuse_Blown_Pin, GPIO_PIN_RESET);
            break;

        case U8_MODE_SELECT:               // Mode select
            push_cmd_u8(APP_CMD_MODE_SELECT, handle_uint8_t_data(data_ptr, data_len));
            break;

        case U16_TREAT_TIME_MIN:           // Treatment time (minutes)
            push_cmd_u16(APP_CMD_SET_TREATMENT_TIME, handle_uint16_t_data(data_ptr, data_len));
            break;

        case U8_START_TREATMENT:           // Start treatment -> start heater/motor control
            push_cmd_u8(APP_CMD_START, 1);
            break;

        case U8_STOP_TREATMENT:            // Stop treatment
            push_cmd_u8(APP_CMD_STOP, 0);
            break;

        case U8_SAVE_SETTINGS:             // Save current parameters
            push_cmd_u8(APP_CMD_SAVE_PARAM, 0);
            break;

        default:                           // Unknown frame
            LOG("unknown frame_id 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}
