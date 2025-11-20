/*
 * UART 协议命令分发（只负责协议 -> app_cmd_t 映射）
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "Uart_Communicate.h"
#include "AppMain/system_app.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "LOG.h"

// 数据解析辅助
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

// 入队便捷函数
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

// 协议帧分发
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        case U8_HEARTBEAT_REQ:
            tx_u8(U8_HEARTBEAT_ACK, 1);
            break;

        case F32_PRESSURE_SET_KPA:
            push_cmd_f32(APP_CMD_SET_PRESSURE_KPA, handle_float_data(data_ptr, data_len));
            break;

        case F32_LEFT_TEMP_SET_C:
            push_cmd_f32(APP_CMD_SET_TEMP, handle_float_data(data_ptr, data_len));
            break;

        case U8_LEFT_EYE_ENABLE:
            push_cmd_u8(APP_CMD_LEFT_ENABLE, handle_uint8_t_data(data_ptr, data_len));
            break;

        case U8_RIGHT_EYE_ENABLE:
            push_cmd_u8(APP_CMD_RIGHT_ENABLE, handle_uint8_t_data(data_ptr, data_len));
            break;

        case U8_MODE_SELECT:
            push_cmd_u8(APP_CMD_MODE_SELECT, handle_uint8_t_data(data_ptr, data_len));
            break;

        case U8_START_TREATMENT:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
            push_cmd_u8(en ? APP_CMD_START : APP_CMD_STOP, en);
            break;
        }

        case U8_STOP_TREATMENT:
            push_cmd_u8(APP_CMD_STOP, 0);
            break;

        case U8_SAVE_SETTINGS:
            push_cmd_u8(APP_CMD_SAVE_PARAM, 0);
            break;

        default:
            LOG("unknown frame_id 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}
