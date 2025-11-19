#include <string.h>
#include "Uart_Communicate.h"
#include "AppMain/system_app.h"
#include "LOG.h"

static uint8_t as_u8(const uint8_t *p, uint16_t len){ uint8_t v=0; if(len>=1) memcpy(&v,p,1); return v; }
static float   as_f32(const uint8_t *p, uint16_t len){ float v=0; if(len>=4) memcpy(&v,p,4); return v; }
static uint16_t as_u16(const uint8_t *p, uint16_t len){ uint16_t v=0; if(len>=2) memcpy(&v,p,2); return v; }
static uint32_t as_u32(const uint8_t *p, uint16_t len){ uint32_t v=0; if(len>=4) memcpy(&v,p,4); return v; }

void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        case U8_HEARTBEAT_REQ:
        {
            tx_frame_t tx = {0}; tx.type = TX_DATA_UINT8; tx.frame_id = U8_HEARTBEAT_ACK; tx.v.u8 = 1;
            xQueueSend(gTxQueue, &tx, 0);
            break;
        }

        case F32_PRESSURE_SET_KPA:
        {
            app_cmd_t cmd = { .id = APP_CMD_SET_PRESSURE_KPA };
            cmd.v.f32 = as_f32(data_ptr, data_len);
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }
        case F32_LEFT_TEMP_SET_C:
        {
            app_cmd_t cmd = { .id = APP_CMD_SET_TEMP_LEFT };
            cmd.v.f32 = as_f32(data_ptr, data_len);
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }
        case F32_RIGHT_TEMP_SET_C:
        {
            app_cmd_t cmd = { .id = APP_CMD_SET_TEMP_RIGHT };
            cmd.v.f32 = as_f32(data_ptr, data_len);
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }

        case U8_MODE_SELECT:
        {
            app_cmd_t cmd = { .id = APP_CMD_MODE_SELECT };
            cmd.v.u8 = as_u8(data_ptr, data_len);
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }
        case U8_START_TREATMENT:
        {
            uint8_t en = as_u8(data_ptr, data_len);
            app_cmd_t cmd = { .id = en ? APP_CMD_START : APP_CMD_STOP };
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }
        case U8_STOP_TREATMENT:
        {
            app_cmd_t cmd = { .id = APP_CMD_STOP };
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }
        case U8_SAVE_SETTINGS:
        {
            app_cmd_t cmd = { .id = APP_CMD_SAVE_PARAM };
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }
        case U8_USE_COMMON_TEMP:
        {
            app_cmd_t cmd = { .id = APP_CMD_SET_USE_COMMON_TEMP };
            cmd.v.u8 = as_u8(data_ptr, data_len);
            xQueueSend(gCmdQueue, &cmd, 0);
            break;
        }

        // Mode1 profile
        case F32_MODE1_TARGET_KPA: { app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=0}; cmd.v.f32=as_f32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U32_MODE1_T_RISE_MS:  { app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=1}; cmd.v.u32=as_u32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U32_MODE1_T_HOLD_MS:  { app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=2}; cmd.v.u32=as_u32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U32_MODE1_T_PULSE_TOTAL_MS:{ app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=3}; cmd.v.u32=as_u32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U16_MODE1_T_PULSE_ON_MS:{ app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=4}; cmd.v.u16=as_u16(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U16_MODE1_T_PULSE_OFF_MS:{ app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=5}; cmd.v.u16=as_u16(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U8_MODE1_SQUEEZE_MODE: { app_cmd_t cmd={.id=APP_CMD_SET_MODE1_PARAM,.key=6}; cmd.v.u8=as_u8(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;

        // Mode2 profile
        case F32_MODE2_TARGET_KPA: { app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=0}; cmd.v.f32=as_f32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U32_MODE2_T_RISE_MS:  { app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=1}; cmd.v.u32=as_u32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U32_MODE2_T_HOLD_MS:  { app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=2}; cmd.v.u32=as_u32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U32_MODE2_T_PULSE_TOTAL_MS:{ app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=3}; cmd.v.u32=as_u32(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U16_MODE2_T_PULSE_ON_MS:{ app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=4}; cmd.v.u16=as_u16(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U16_MODE2_T_PULSE_OFF_MS:{ app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=5}; cmd.v.u16=as_u16(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;
        case U8_MODE2_SQUEEZE_MODE: { app_cmd_t cmd={.id=APP_CMD_SET_MODE2_PARAM,.key=6}; cmd.v.u8=as_u8(data_ptr,data_len); xQueueSend(gCmdQueue,&cmd,0);} break;

        default:
            LOG("unknown frame_id 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}

