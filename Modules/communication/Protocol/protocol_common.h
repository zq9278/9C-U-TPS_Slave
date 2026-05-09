#ifndef MODULES_COMMUNICATION_PROTOCOL_COMMON_H
#define MODULES_COMMUNICATION_PROTOCOL_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include "Modules/communication/Protocol/protocol_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RK3576 协议固定帧格式常量。 */
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

/* 对一帧已通过基础校验的协议数据提供只读视图。 */
typedef struct
{
    uint16_t frame_id;
    uint8_t data_type;
    uint16_t payload_length;
    const uint8_t *payload;
} ProtocolFrameView;

/* 底层发送字节流的抽象回调。 */
typedef uint8_t (*ProtocolSendBytesCallback)(void *context,
                                             const uint8_t *data,
                                             size_t length);

/* 协议层抽象传输接口。 */
typedef struct
{
    ProtocolSendBytesCallback send_bytes;
    void *send_context;
} ProtocolTransport;

/* 流式收包状态机上下文。 */
typedef struct
{
    uint8_t buffer[PROTOCOL_FRAME_MAX_LENGTH];
    uint16_t length;
    uint16_t expected_length;
    uint8_t collecting;
} ProtocolRxState;

/* 计算一段数据的 Modbus CRC16。 */
uint16_t Protocol_Crc16Modbus(const uint8_t *data, size_t length);

/* 构建完整协议帧字节流。 */
uint8_t Protocol_BuildFrame(uint16_t frame_id,
                            uint8_t data_type,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *out_buffer,
                            uint16_t *out_length);

/* 使用 transport 发送一帧协议。 */
uint8_t Protocol_SendFrame(const ProtocolTransport *transport,
                           uint16_t frame_id,
                           uint8_t data_type,
                           const uint8_t *payload,
                           uint16_t payload_length);

#ifdef __cplusplus
}
#endif

#endif
