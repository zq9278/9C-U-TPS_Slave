#ifndef MODULES_COMMUNICATION_COMMUNICATION_H
#define MODULES_COMMUNICATION_COMMUNICATION_H

#include <stddef.h>
#include <stdint.h>
#include "Modules/communication/ascii_processor.h"
#include "Modules/communication/Protocol/protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    COMM_INTERFACE_LOG_UART1 = 0,
    COMM_INTERFACE_RK3576_UART3 = 1,
    COMM_INTERFACE_COUNT
} CommunicationInterfaceId;

typedef ProtocolFrameView CommunicationFrameView;

typedef void (*CommunicationProtocolFrameCallback)(void *context,
                                                   CommunicationInterfaceId interface_id,
                                                   const CommunicationFrameView *frame);
typedef void (*CommunicationAsciiCommandCallback)(void *context,
                                                  CommunicationChannel channel,
                                                  const char *line,
                                                  size_t length);

typedef struct
{
    CommunicationProtocolFrameCallback on_protocol_frame;
    CommunicationAsciiCommandCallback on_ascii_command;
    void *context;
} CommunicationCallbacks;

void Communication_Init(void);
void Communication_SetCallbacks(const CommunicationCallbacks *callbacks);
void Communication_PollRx(void);

uint8_t Communication_SendFrame(CommunicationInterfaceId interface_id,
                                uint16_t frame_id,
                                uint8_t data_type,
                                const void *payload,
                                uint16_t payload_length);
uint8_t Communication_SendU8(uint16_t frame_id, uint8_t value);
uint8_t Communication_SendU16(uint16_t frame_id, uint16_t value);
uint8_t Communication_SendU32(uint16_t frame_id, uint32_t value);
uint8_t Communication_SendF32(uint16_t frame_id, float value);
uint8_t Communication_SendText(uint16_t frame_id, const char *text);

void Communication_WriteLog(const char *text);
int Communication_WriteLogBuffer(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
