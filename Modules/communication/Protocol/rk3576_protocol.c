#include "Modules/communication/Protocol/rk3576_protocol.h"

#include <string.h>

/* 读取 1 字节 payload。 */
static uint8_t frame_get_u8(const ProtocolFrameView *frame, uint8_t *out)
{
    if ((frame == NULL) || (out == NULL) || (frame->payload_length < sizeof(uint8_t)))
    {
        return 0U;
    }

    *out = frame->payload[0];
    return 1U;
}

/* 读取 2 字节 payload。 */
static uint8_t frame_get_u16(const ProtocolFrameView *frame, uint16_t *out)
{
    if ((frame == NULL) || (out == NULL) || (frame->payload_length < sizeof(uint16_t)))
    {
        return 0U;
    }

    (void)memcpy(out, frame->payload, sizeof(uint16_t));
    return 1U;
}

/* 读取 4 字节 float payload。 */
static uint8_t frame_get_f32(const ProtocolFrameView *frame, float *out)
{
    if ((frame == NULL) || (out == NULL) || (frame->payload_length < sizeof(float)))
    {
        return 0U;
    }

    (void)memcpy(out, frame->payload, sizeof(float));
    return 1U;
}

uint8_t Rk3576Protocol_FrameToAppCommand(const ProtocolFrameView *frame,
                                         app_cmd_t *command)
{
    if ((frame == NULL) || (command == NULL))
    {
        return 0U;
    }

    (void)memset(command, 0, sizeof(*command));

    /* 这里只做“协议帧 -> 应用命令”的纯翻译，不直接操作硬件。 */
    switch (frame->frame_id)
    {
    case PROTOCOL_ID_U8_HEARTBEAT_REQ:
        command->id = APP_CMD_HEARTBEAT;
        break;
    case PROTOCOL_ID_F32_PRESSURE_SET_KPA:
        command->id = APP_CMD_SET_PRESSURE_KPA;
        if (frame_get_f32(frame, &command->v.f32) == 0U) { return 0U; }
        break;
    case PROTOCOL_ID_F32_LEFT_TEMP_SET_C:
        command->id = APP_CMD_SET_TEMP;
        if (frame_get_f32(frame, &command->v.f32) == 0U) { return 0U; }
        break;
    case PROTOCOL_ID_U8_LEFT_EYE_ENABLE:
        command->id = APP_CMD_LEFT_ENABLE;
        if (frame_get_u8(frame, &command->v.u8) == 0U) { return 0U; }
        break;
    case PROTOCOL_ID_U8_RIGHT_EYE_ENABLE:
        command->id = APP_CMD_RIGHT_ENABLE;
        if (frame_get_u8(frame, &command->v.u8) == 0U) { return 0U; }
        break;
    case PROTOCOL_ID_U8_LEFT_HEATER_FUSE_BLOW_CMD:
        command->id = APP_CMD_LEFT_HEATER_FUSE_BLOW;
        break;
    case PROTOCOL_ID_U8_RIGHT_HEATER_FUSE_BLOW_CMD:
        command->id = APP_CMD_RIGHT_HEATER_FUSE_BLOW;
        break;
    case PROTOCOL_ID_U16_TREAT_TIME_MIN:
        command->id = APP_CMD_SET_TREATMENT_TIME;
        if (frame_get_u16(frame, &command->v.u16) == 0U) { return 0U; }
        break;
    case PROTOCOL_ID_U8_MODE_SELECT:
        command->id = APP_CMD_MODE_SELECT;
        if (frame_get_u8(frame, &command->v.u8) == 0U) { return 0U; }
        break;
    case PROTOCOL_ID_U8_START_TREATMENT:
        command->id = APP_CMD_START;
        break;
    case PROTOCOL_ID_U8_STOP_TREATMENT:
        command->id = APP_CMD_STOP;
        break;
    case PROTOCOL_ID_U8_SAVE_SETTINGS:
        command->id = APP_CMD_SAVE_PARAM;
        break;
    case PROTOCOL_ID_U8_PAUSE_RESUME_TREATMENT:
        command->id = APP_CMD_PAUSE_RESUME;
        if (frame_get_u8(frame, &command->v.u8) == 0U) { return 0U; }
        break;
    default:
        return 0U;
    }

    return 1U;
}
