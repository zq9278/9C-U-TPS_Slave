#ifndef MODULES_COMMUNICATION_PROTOCOL_PID_DEBUG_PROTOCOL_H
#define MODULES_COMMUNICATION_PROTOCOL_PID_DEBUG_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "App/System/system_app.h"
#include "Modules/Pid/pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PID 调试下行命令抽象。 */
typedef struct
{
    ctrl_cmd_id_t id;
    pid_debug_target_t target;
    float kp;
    float ki;
    float kd;
    uint8_t enabled;
} PidDebugCommand;

/* PID 调试串口接收状态机。 */
typedef struct
{
    uint8_t state;
    uint8_t command_id;
    uint8_t payload_length;
    uint8_t payload_index;
    uint8_t payload[16];
    uint8_t crc_lo;
} PidDebugRxParser;

/* 初始化 PID 调试协议接收状态。 */
void PidDebugProtocol_InitRxParser(PidDebugRxParser *parser);
/* 输入一个字节，若凑成完整命令则返回 1。 */
uint8_t PidDebugProtocol_ProcessRxByte(PidDebugRxParser *parser,
                                       uint8_t byte,
                                       PidDebugCommand *command);
/* 发送一帧 PID 调试采样。 */
uint8_t PidDebugProtocol_SendSample(pid_debug_target_t target,
                                    uint32_t tick_ms,
                                    const PidController *pid);

#ifdef __cplusplus
}
#endif

#endif
