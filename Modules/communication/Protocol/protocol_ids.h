#ifndef MODULES_PROTOCOL_PROTOCOL_IDS_H
#define MODULES_PROTOCOL_PROTOCOL_IDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH = 32U
};

typedef enum
{
    PROTOCOL_DATA_TYPE_NONE = 0x00,
    PROTOCOL_DATA_TYPE_UINT8 = 0x01,
    PROTOCOL_DATA_TYPE_FLOAT = 0x02,
    PROTOCOL_DATA_TYPE_TEXT = 0x03,
    PROTOCOL_DATA_TYPE_UINT16 = 0x04,
    PROTOCOL_DATA_TYPE_UINT32 = 0x05
} ProtocolDataType;

/*
 * Direction note:
 *
 * 1. RK3576 -> STM32
 *    These are active control commands sent from RK3576 to STM32.
 *    The current project uses the 0x1000 ~ 0x10C4 group for this direction.
 *
 * 2. STM32 -> RK3576
 *    These are replies / telemetry / status frames sent from STM32 to RK3576.
 *    The current project uses the 0x1100 ~ 0x110C group for this direction.
 *
 * 3. Compatibility quirk
 *    The current protocol also reuses some 0x100x IDs for device-to-host
 *    setting synchronization. That compatibility behavior is kept in the
 *    dispatch layer and should be treated as a special case, not as the
 *    primary direction definition.
 */
typedef enum
{
    /* RK3576 -> STM32 */
    PROTOCOL_ID_U8_HEARTBEAT_REQ = 0x1000,
    PROTOCOL_ID_F32_PRESSURE_SET_KPA = 0x1001,
    PROTOCOL_ID_F32_LEFT_TEMP_SET_C = 0x1002,
    PROTOCOL_ID_U8_LEFT_EYE_ENABLE = 0x1004,
    PROTOCOL_ID_U8_RIGHT_EYE_ENABLE = 0x1005,
    PROTOCOL_ID_U16_TREAT_TIME_MIN = 0x1006,
    PROTOCOL_ID_U8_LEFT_HEATER_FUSE_BLOW_CMD = 0x1007,
    PROTOCOL_ID_U8_RIGHT_HEATER_FUSE_BLOW_CMD = 0x1008,
    PROTOCOL_ID_U8_MODE_SELECT = 0x10C0,
    PROTOCOL_ID_U8_START_TREATMENT = 0x10C1,
    PROTOCOL_ID_U8_STOP_TREATMENT = 0x10C2,
    PROTOCOL_ID_U8_SAVE_SETTINGS = 0x10C3,
    PROTOCOL_ID_U8_PAUSE_RESUME_TREATMENT = 0x10C4,

    /* STM32 -> RK3576 */
    PROTOCOL_ID_U8_HEARTBEAT_ACK = 0x1100,
    PROTOCOL_ID_F32_LEFT_PRESSURE_VALUE = 0x1101,
    PROTOCOL_ID_F32_RIGHT_PRESSURE_VALUE = 0x1102,
    PROTOCOL_ID_F32_LEFT_TEMP_VALUE = 0x1103,
    PROTOCOL_ID_F32_RIGHT_TEMP_VALUE = 0x1104,
    PROTOCOL_ID_U8_LEFT_HEATER_PRESENT = 0x1107,
    PROTOCOL_ID_U8_RIGHT_HEATER_PRESENT = 0x1108,
    PROTOCOL_ID_U8_LEFT_HEATER_FUSE = 0x1109,
    PROTOCOL_ID_U8_RIGHT_HEATER_FUSE = 0x110A,
    PROTOCOL_ID_U8_MODE_CURVES = 0x110B,
    PROTOCOL_ID_U8_STOP_REASON = 0x110C
} ProtocolFrameId;

static inline uint8_t Protocol_IsRk3576ToStm32Command(uint16_t frame_id)
{
    /**
     * 这里做的是“方向判断”，不是业务分发。
     * 只要 frame_id 属于 RK3576 发给 STM32 的命令集合，就返回 1。
     */
    switch (frame_id)
    {
    case PROTOCOL_ID_U8_HEARTBEAT_REQ:
    case PROTOCOL_ID_F32_PRESSURE_SET_KPA:
    case PROTOCOL_ID_F32_LEFT_TEMP_SET_C:
    case PROTOCOL_ID_U8_LEFT_EYE_ENABLE:
    case PROTOCOL_ID_U8_RIGHT_EYE_ENABLE:
    case PROTOCOL_ID_U16_TREAT_TIME_MIN:
    case PROTOCOL_ID_U8_LEFT_HEATER_FUSE_BLOW_CMD:
    case PROTOCOL_ID_U8_RIGHT_HEATER_FUSE_BLOW_CMD:
    case PROTOCOL_ID_U8_MODE_SELECT:
    case PROTOCOL_ID_U8_START_TREATMENT:
    case PROTOCOL_ID_U8_STOP_TREATMENT:
    case PROTOCOL_ID_U8_SAVE_SETTINGS:
    case PROTOCOL_ID_U8_PAUSE_RESUME_TREATMENT:
        return 1U;

    default:
        return 0U;
    }
}

static inline uint8_t Protocol_IsStm32ToRk3576Status(uint16_t frame_id)
{
    /**
     * 这里判断一帧是否属于 STM32 发给 RK3576 的状态、回包或遥测集合。
     */
    switch (frame_id)
    {
    case PROTOCOL_ID_U8_HEARTBEAT_ACK:
    case PROTOCOL_ID_F32_LEFT_PRESSURE_VALUE:
    case PROTOCOL_ID_F32_RIGHT_PRESSURE_VALUE:
    case PROTOCOL_ID_F32_LEFT_TEMP_VALUE:
    case PROTOCOL_ID_F32_RIGHT_TEMP_VALUE:
    case PROTOCOL_ID_U8_LEFT_HEATER_PRESENT:
    case PROTOCOL_ID_U8_RIGHT_HEATER_PRESENT:
    case PROTOCOL_ID_U8_LEFT_HEATER_FUSE:
    case PROTOCOL_ID_U8_RIGHT_HEATER_FUSE:
    case PROTOCOL_ID_U8_MODE_CURVES:
    case PROTOCOL_ID_U8_STOP_REASON:
        return 1U;

    default:
        return 0U;
    }
}

#ifdef __cplusplus
}
#endif

#endif
