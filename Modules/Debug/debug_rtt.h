#ifndef TPS_DEBUG_RTT_H
#define TPS_DEBUG_RTT_H
#include <stddef.h>
void DebugRtt_Init(void);
int DebugRtt_Write(const void *data, size_t size);
int DebugRtt_Telemetry(const void *data, size_t size);
/* Polled by CommRxTask; PID requests execute through the ControlTask queue. */
void DebugRtt_Poll(void);
#endif
