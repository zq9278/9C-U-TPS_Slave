/*
 * UART 协议命令分发（UTF-8）
 * 仅负责：将解析出的帧ID+数据，转换为 AppTask 能理解的命令（app_cmd_t）并入队。
 * 不做：任何硬件直控/定时器/保存操作；心跳应答等下行通过 gTxQueue 统一发送。
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "Uart_Communicate.h"
#include "AppMain/system_app.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "LOG.h"

// ---------------- 数据解码辅助 ----------------
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len)
{
    (void)data_ptr; (void)data_len; // 预留
}

uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    uint8_t v = 0; if (data_len >= 1) memcpy(&v, data_ptr, 1); return v;
}

float handle_float_data(const uint8_t *data_ptr, uint16_t data_len)
{
    float v = 0.0f; if (data_len >= sizeof(float)) memcpy(&v, data_ptr, sizeof(float)); return v;
}

static inline uint16_t read_u16_le(const uint8_t *p, uint16_t len)
{ uint16_t v=0; if (len>=2) memcpy(&v,p,2); return v; }
static inline uint32_t read_u32_le(const uint8_t *p, uint16_t len)
{ uint32_t v=0; if (len>=4) memcpy(&v,p,4); return v; }

// ---------------- 入队便捷函数 ----------------
static inline void tx_u8(uint16_t id, uint8_t v)
{
    tx_frame_t tx = {0}; tx.type = TX_DATA_UINT8; tx.frame_id = id; tx.v.u8 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static inline void push_cmd_u8(app_cmd_id_t id, uint8_t v)
{ app_cmd_t c={0}; c.id=id; c.v.u8=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_f32(app_cmd_id_t id, float v)
{ app_cmd_t c={0}; c.id=id; c.v.f32=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_u8(app_cmd_id_t id, uint16_t key, uint8_t v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.u8=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_u16(app_cmd_id_t id, uint16_t key, uint16_t v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.u16=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_u32(app_cmd_id_t id, uint16_t key, uint32_t v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.u32=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_f32(app_cmd_id_t id, uint16_t key, float v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.f32=v; (void)xQueueSend(gCmdQueue,&c,0); }

// ---------------- 协议帧分发 ----------------
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        // 基础通信
        case U8_HEARTBEAT_REQ:
            tx_u8(U8_HEARTBEAT_ACK, 1);
            break;

        // 参数设定
        case F32_PRESSURE_SET_KPA:
            push_cmd_f32(APP_CMD_SET_PRESSURE_KPA, handle_float_data(data_ptr, data_len));
            break;
        case F32_LEFT_TEMP_SET_C:
            push_cmd_f32(APP_CMD_SET_TEMP_LEFT, handle_float_data(data_ptr, data_len));
            break;
        case F32_RIGHT_TEMP_SET_C:
            push_cmd_f32(APP_CMD_SET_TEMP_RIGHT, handle_float_data(data_ptr, data_len));
            break;
        case U8_USE_COMMON_TEMP:
            push_cmd_u8(APP_CMD_SET_USE_COMMON_TEMP, handle_uint8_t_data(data_ptr, data_len));
            break;

        // 模式/流程控制
        case U8_MODE_SELECT:
            push_cmd_u8(APP_CMD_MODE_SELECT, handle_uint8_t_data(data_ptr, data_len));
            break;
        case U8_START_TREATMENT:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
            app_cmd_t c={0}; c.id = en ? APP_CMD_START : APP_CMD_STOP; (void)xQueueSend(gCmdQueue,&c,0);
            break;
        }
        case U8_STOP_TREATMENT:
        {
            app_cmd_t c={0}; c.id = APP_CMD_STOP; (void)xQueueSend(gCmdQueue,&c,0);
            break;
        }
        case U8_SAVE_SETTINGS:
        {
            app_cmd_t c={0}; c.id = APP_CMD_SAVE_PARAM; (void)xQueueSend(gCmdQueue,&c,0);
            break;
        }

        // 模式1曲线
        case F32_MODE1_TARGET_KPA:       push_cmd_key_f32(APP_CMD_SET_MODE1_PARAM, 0, handle_float_data(data_ptr, data_len)); break;
        case U32_MODE1_T_RISE_MS:        push_cmd_key_u32(APP_CMD_SET_MODE1_PARAM, 1, read_u32_le(data_ptr, data_len));      break;
        case U32_MODE1_T_HOLD_MS:        push_cmd_key_u32(APP_CMD_SET_MODE1_PARAM, 2, read_u32_le(data_ptr, data_len));      break;
        case U32_MODE1_T_PULSE_TOTAL_MS: push_cmd_key_u32(APP_CMD_SET_MODE1_PARAM, 3, read_u32_le(data_ptr, data_len));      break;
        case U16_MODE1_T_PULSE_ON_MS:    push_cmd_key_u16(APP_CMD_SET_MODE1_PARAM, 4, read_u16_le(data_ptr, data_len));      break;
        case U16_MODE1_T_PULSE_OFF_MS:   push_cmd_key_u16(APP_CMD_SET_MODE1_PARAM, 5, read_u16_le(data_ptr, data_len));      break;
        case U8_MODE1_SQUEEZE_MODE:      push_cmd_key_u8 (APP_CMD_SET_MODE1_PARAM, 6, handle_uint8_t_data(data_ptr, data_len)); break;

        // 模式2曲线
        case F32_MODE2_TARGET_KPA:       push_cmd_key_f32(APP_CMD_SET_MODE2_PARAM, 0, handle_float_data(data_ptr, data_len)); break;
        case U32_MODE2_T_RISE_MS:        push_cmd_key_u32(APP_CMD_SET_MODE2_PARAM, 1, read_u32_le(data_ptr, data_len));      break;
        case U32_MODE2_T_HOLD_MS:        push_cmd_key_u32(APP_CMD_SET_MODE2_PARAM, 2, read_u32_le(data_ptr, data_len));      break;
        case U32_MODE2_T_PULSE_TOTAL_MS: push_cmd_key_u32(APP_CMD_SET_MODE2_PARAM, 3, read_u32_le(data_ptr, data_len));      break;
        case U16_MODE2_T_PULSE_ON_MS:    push_cmd_key_u16(APP_CMD_SET_MODE2_PARAM, 4, read_u16_le(data_ptr, data_len));      break;
        case U16_MODE2_T_PULSE_OFF_MS:   push_cmd_key_u16(APP_CMD_SET_MODE2_PARAM, 5, read_u16_le(data_ptr, data_len));      break;
        case U8_MODE2_SQUEEZE_MODE:      push_cmd_key_u8 (APP_CMD_SET_MODE2_PARAM, 6, handle_uint8_t_data(data_ptr, data_len)); break;

        // 文本/未知
        case FRAME_ID_TEXT:
            LOG("TEXT len=%d\r\n", data_len);
            break;

        default:
            LOG("unknown frame_id 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}

