#ifndef MODULES_COMMUNICATION_PROTOCOL_COMMON_H
#define MODULES_COMMUNICATION_PROTOCOL_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include "Modules/communication/Protocol/protocol_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    PROTOCOL_FRAME_HEADER_1 = 0xAAU,
    PROTOCOL_FRAME_HEADER_2 = 0x55U,
    PROTOCOL_FRAME_TAIL_1 = 0x0DU,
    PROTOCOL_FRAME_TAIL_2 = 0x0AU,
    PROTOCOL_FRAME_OVERHEAD = 11U,
    PROTOCOL_FRAME_MAX_LENGTH =
        PROTOCOL_FRAME_OVERHEAD + PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH
};

typedef struct
{
    uint16_t frame_id;
    uint8_t data_type;
    uint16_t payload_length;
    const uint8_t *payload;
} ProtocolFrameView;

typedef uint8_t (*ProtocolSendBytesCallback)(void *context,
                                             const uint8_t *data,
                                             size_t length);

typedef struct
{
    ProtocolSendBytesCallback send_bytes;
    void *send_context;
} ProtocolTransport;

typedef struct
{
    uint8_t buffer[PROTOCOL_FRAME_MAX_LENGTH];
    uint16_t length;
    uint16_t expected_length;
    uint8_t collecting;
} ProtocolRxState;

uint16_t Protocol_Crc16Modbus(const uint8_t *data, size_t length);

uint8_t Protocol_BuildFrame(uint16_t frame_id,
                            uint8_t data_type,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *out_buffer,
                            uint16_t *out_length);

uint8_t Protocol_SendFrame(const ProtocolTransport *transport,
                           uint16_t frame_id,
                           uint8_t data_type,
                           const uint8_t *payload,
                           uint16_t payload_length);

#ifdef __cplusplus
}
#endif

#endif
