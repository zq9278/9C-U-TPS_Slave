#ifndef TPS_DEBUG_RTT_H
#define TPS_DEBUG_RTT_H
#include <stddef.h>
void DebugRtt_Init(void);
int DebugRtt_Write(const void *data, size_t size);
int DebugRtt_Telemetry(const void *data, size_t size);
/* Read-only commands, polled by CommRxTask. Never runs treatment commands. */
void DebugRtt_Poll(void);
#endif
