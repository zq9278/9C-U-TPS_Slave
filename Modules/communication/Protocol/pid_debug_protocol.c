#include "Modules/communication/Protocol/pid_debug_protocol.h"

#include <string.h>

#include "Modules/communication/communication.h"
#include "Modules/communication/Protocol/protocol_common.h"

/* PID 调试协议帧格式常量。 */
#define PID_DEBUG_FRAME_MAGIC_1             0xC5U
#define PID_DEBUG_FRAME_MAGIC_2             0x5CU
#define PID_DEBUG_FRAME_VERSION             0x01U
#define PID_DEBUG_FRAME_LENGTH              56U

#define PID_DEBUG_CMD_SET_GAINS             0x01U
#define PID_DEBUG_CMD_STREAM_ENABLE         0x02U

#define PID_DEBUG_CMD_SET_GAINS_PAYLOAD_LEN 13U
#define PID_DEBUG_CMD_STREAM_PAYLOAD_LEN    1U

enum
{
    PID_DEBUG_RX_WAIT_MAGIC_1 = 0,
    PID_DEBUG_RX_WAIT_MAGIC_2,
    PID_DEBUG_RX_WAIT_VERSION,
    PID_DEBUG_RX_WAIT_COMMAND,
    PID_DEBUG_RX_WAIT_LENGTH,
    PID_DEBUG_RX_WAIT_PAYLOAD,
    PID_DEBUG_RX_WAIT_CRC_LO,
    PID_DEBUG_RX_WAIT_CRC_HI
};

/* 复位 PID 调试协议接收状态机。 */
static void PidDebugProtocol_ResetParser(PidDebugRxParser *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->state = PID_DEBUG_RX_WAIT_MAGIC_1;
    parser->command_id = 0U;
    parser->payload_length = 0U;
    parser->payload_index = 0U;
    parser->crc_lo = 0U;
    (void)memset(parser->payload, 0, sizeof(parser->payload));
}

/* 校验调试目标是否合法。 */
static uint8_t PidDebugProtocol_IsValidTarget(uint8_t target)
{
    return (uint8_t)(target <= (uint8_t)PID_DEBUG_TARGET_HEAT_RIGHT);
}

/* 把原始 payload 解码为上层控制命令。 */
static uint8_t PidDebugProtocol_DecodeCommand(uint8_t command_id,
                                              const uint8_t *payload,
                                              uint8_t payload_length,
                                              PidDebugCommand *command)
{
    if ((payload == NULL) || (command == NULL))
    {
        return 0U;
    }

    (void)memset(command, 0, sizeof(*command));

    if (command_id == PID_DEBUG_CMD_STREAM_ENABLE)
    {
        if (payload_length != PID_DEBUG_CMD_STREAM_PAYLOAD_LEN)
        {
            return 0U;
        }

        command->id = CTRL_CMD_PID_STREAM_ENABLE;
        command->enabled = (payload[0] != 0U) ? 1U : 0U;
        return 1U;
    }

    if (command_id == PID_DEBUG_CMD_SET_GAINS)
    {
        if ((payload_length != PID_DEBUG_CMD_SET_GAINS_PAYLOAD_LEN) ||
            (PidDebugProtocol_IsValidTarget(payload[0]) == 0U))
        {
            return 0U;
        }

        command->id = CTRL_CMD_PID_SET_GAINS;
        command->target = (pid_debug_target_t)payload[0];
        (void)memcpy(&command->kp, &payload[1], sizeof(float));
        (void)memcpy(&command->ki, &payload[5], sizeof(float));
        (void)memcpy(&command->kd, &payload[9], sizeof(float));
        return 1U;
    }

    return 0U;
}

void PidDebugProtocol_InitRxParser(PidDebugRxParser *parser)
{
    PidDebugProtocol_ResetParser(parser);
}

uint8_t PidDebugProtocol_ProcessRxByte(PidDebugRxParser *parser,
                                       uint8_t byte,
                                       PidDebugCommand *command)
{
    uint8_t frame_data[19];
    uint16_t expected_crc;
    uint16_t received_crc;

    if ((parser == NULL) || (command == NULL))
    {
        return 0U;
    }

    /* 字节流状态机，适配串口的不定长调试帧。 */
    switch (parser->state)
    {
    case PID_DEBUG_RX_WAIT_MAGIC_1:
        if (byte == PID_DEBUG_FRAME_MAGIC_1)
        {
            parser->state = PID_DEBUG_RX_WAIT_MAGIC_2;
        }
        break;

    case PID_DEBUG_RX_WAIT_MAGIC_2:
        if (byte == PID_DEBUG_FRAME_MAGIC_2)
        {
            parser->state = PID_DEBUG_RX_WAIT_VERSION;
        }
        else if (byte != PID_DEBUG_FRAME_MAGIC_1)
        {
            PidDebugProtocol_ResetParser(parser);
        }
        break;

    case PID_DEBUG_RX_WAIT_VERSION:
        if (byte == PID_DEBUG_FRAME_VERSION)
        {
            parser->state = PID_DEBUG_RX_WAIT_COMMAND;
        }
        else
        {
            PidDebugProtocol_ResetParser(parser);
        }
        break;

    case PID_DEBUG_RX_WAIT_COMMAND:
        parser->command_id = byte;
        parser->state = PID_DEBUG_RX_WAIT_LENGTH;
        break;

    case PID_DEBUG_RX_WAIT_LENGTH:
        parser->payload_length = byte;
        parser->payload_index = 0U;
        if (parser->payload_length > sizeof(parser->payload))
        {
            PidDebugProtocol_ResetParser(parser);
        }
        else if (parser->payload_length == 0U)
        {
            parser->state = PID_DEBUG_RX_WAIT_CRC_LO;
        }
        else
        {
            parser->state = PID_DEBUG_RX_WAIT_PAYLOAD;
        }
        break;

    case PID_DEBUG_RX_WAIT_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;
        if (parser->payload_index >= parser->payload_length)
        {
            parser->state = PID_DEBUG_RX_WAIT_CRC_LO;
        }
        break;

    case PID_DEBUG_RX_WAIT_CRC_LO:
        parser->crc_lo = byte;
        parser->state = PID_DEBUG_RX_WAIT_CRC_HI;
        break;

    case PID_DEBUG_RX_WAIT_CRC_HI:
        frame_data[0] = PID_DEBUG_FRAME_VERSION;
        frame_data[1] = parser->command_id;
        frame_data[2] = parser->payload_length;
        if (parser->payload_length > 0U)
        {
            (void)memcpy(&frame_data[3], parser->payload, parser->payload_length);
        }

        expected_crc = Protocol_Crc16Modbus(frame_data, (size_t)(3U + parser->payload_length));
        received_crc = (uint16_t)parser->crc_lo | ((uint16_t)byte << 8);
        if ((expected_crc == received_crc) &&
            (PidDebugProtocol_DecodeCommand(parser->command_id,
                                            parser->payload,
                                            parser->payload_length,
                                            command) != 0U))
        {
            PidDebugProtocol_ResetParser(parser);
            return 1U;
        }

        PidDebugProtocol_ResetParser(parser);
        break;

    default:
        PidDebugProtocol_ResetParser(parser);
        break;
    }

    return 0U;
}

uint8_t PidDebugProtocol_SendSample(pid_debug_target_t target,
                                    uint32_t tick_ms,
                                    const PidController *pid)
{
    uint8_t frame[PID_DEBUG_FRAME_LENGTH];
    uint16_t crc;
    uint8_t *cursor = frame;

    if (pid == NULL)
    {
        return 0U;
    }

    /* 顺序打包：标识头、版本、目标、tick、PID 配置和本次计算结果。 */
    *cursor++ = PID_DEBUG_FRAME_MAGIC_1;
    *cursor++ = PID_DEBUG_FRAME_MAGIC_2;
    *cursor++ = PID_DEBUG_FRAME_VERSION;
    *cursor++ = (uint8_t)target;
    (void)memcpy(cursor, &tick_ms, sizeof(tick_ms));
    cursor += sizeof(tick_ms);
    (void)memcpy(cursor, &pid->config.kp, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->config.ki, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->config.kd, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.p, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.i, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.d, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.error, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.output, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.mapped_output, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.setpoint, sizeof(float));
    cursor += sizeof(float);
    (void)memcpy(cursor, &pid->debug.measurement, sizeof(float));
    cursor += sizeof(float);

    crc = Protocol_Crc16Modbus(&frame[2], (size_t)(cursor - &frame[2]));
    (void)memcpy(cursor, &crc, sizeof(crc));
    cursor += sizeof(crc);
    *cursor++ = '\r';
    *cursor++ = '\n';

    return (Communication_WriteLogBuffer(frame, sizeof(frame)) == (int)sizeof(frame)) ? 1U : 0U;
}
