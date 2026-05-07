#include "Modules/Pid/pid_debug_protocol.h"

#include <stdlib.h>
#include <string.h>

#include "Modules/communication/communication.h"
#include "Modules/communication/Protocol/protocol_common.h"

#define PID_DEBUG_FRAME_MAGIC_1 0xC5U
#define PID_DEBUG_FRAME_MAGIC_2 0x5CU
#define PID_DEBUG_FRAME_VERSION 0x01U
#define PID_DEBUG_FRAME_LENGTH  52U

static uint8_t PidDebugProtocol_ParseTarget(const char *text, pid_debug_target_t *target)
{
    if ((text == NULL) || (target == NULL))
    {
        return 0U;
    }

    if (strcmp(text, "PR") == 0)
    {
        *target = PID_DEBUG_TARGET_PRESS_RISE;
        return 1U;
    }
    if (strcmp(text, "PH") == 0)
    {
        *target = PID_DEBUG_TARGET_PRESS_HOLD;
        return 1U;
    }
    if (strcmp(text, "PP") == 0)
    {
        *target = PID_DEBUG_TARGET_PRESS_PULSE;
        return 1U;
    }
    if (strcmp(text, "HL") == 0)
    {
        *target = PID_DEBUG_TARGET_HEAT_LEFT;
        return 1U;
    }
    if (strcmp(text, "HR") == 0)
    {
        *target = PID_DEBUG_TARGET_HEAT_RIGHT;
        return 1U;
    }

    return 0U;
}

const char *PidDebugProtocol_TargetName(pid_debug_target_t target)
{
    switch (target)
    {
    case PID_DEBUG_TARGET_PRESS_RISE:
        return "PR";
    case PID_DEBUG_TARGET_PRESS_HOLD:
        return "PH";
    case PID_DEBUG_TARGET_PRESS_PULSE:
        return "PP";
    case PID_DEBUG_TARGET_HEAT_LEFT:
        return "HL";
    case PID_DEBUG_TARGET_HEAT_RIGHT:
        return "HR";
    default:
        return "UK";
    }
}

uint8_t PidDebugProtocol_ParseCommand(const char *line,
                                      size_t length,
                                      PidDebugCommand *command)
{
    char local[80];
    char *token;
    char *tokens[6];
    size_t token_count = 0U;

    if ((line == NULL) || (command == NULL) || (length == 0U))
    {
        return 0U;
    }

    if (length >= sizeof(local))
    {
        length = sizeof(local) - 1U;
    }
    (void)memcpy(local, line, length);
    local[length] = '\0';

    (void)memset(command, 0, sizeof(*command));

    token = strtok(local, " \t");
    while ((token != NULL) && (token_count < (sizeof(tokens) / sizeof(tokens[0]))))
    {
        tokens[token_count++] = token;
        token = strtok(NULL, " \t");
    }

    if ((token_count < 2U) || (strcmp(tokens[0], "PID") != 0))
    {
        return 0U;
    }

    if (strcmp(tokens[1], "STREAM") == 0)
    {
        if (token_count < 3U)
        {
            return 0U;
        }

        command->id = CTRL_CMD_PID_STREAM_ENABLE;
        command->enabled = (strtoul(tokens[2], NULL, 10) != 0UL) ? 1U : 0U;
        return 1U;
    }

    if (strcmp(tokens[1], "SET") == 0)
    {
        if (token_count < 6U)
        {
            return 0U;
        }

        if (PidDebugProtocol_ParseTarget(tokens[2], &command->target) == 0U)
        {
            return 0U;
        }

        command->kp = strtof(tokens[3], NULL);
        command->ki = strtof(tokens[4], NULL);
        command->kd = strtof(tokens[5], NULL);
        command->id = CTRL_CMD_PID_SET_GAINS;
        return 1U;
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
