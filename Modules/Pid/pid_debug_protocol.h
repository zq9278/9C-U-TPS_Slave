#ifndef MODULES_PID_PID_DEBUG_PROTOCOL_H
#define MODULES_PID_PID_DEBUG_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "App/System/system_app.h"
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    ctrl_cmd_id_t id;
    pid_debug_target_t target;
    float kp;
    float ki;
    float kd;
    uint8_t enabled;
} PidDebugCommand;

uint8_t PidDebugProtocol_ParseCommand(const char *line,
                                      size_t length,
                                      PidDebugCommand *command);
uint8_t PidDebugProtocol_SendSample(pid_debug_target_t target,
                                    uint32_t tick_ms,
                                    const PidController *pid);
const char *PidDebugProtocol_TargetName(pid_debug_target_t target);

#ifdef __cplusplus
}
#endif

#endif
