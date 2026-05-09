#ifndef MODULES_COMMUNICATION_COMMUNICATION_H
#define MODULES_COMMUNICATION_COMMUNICATION_H

#include <stddef.h>
#include <stdint.h>
#include "Modules/communication/Protocol/protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 当前通信模块管理的通道枚举。 */
typedef enum
{
    COMM_CHANNEL_UART1 = 0,
    COMM_CHANNEL_UART3 = 1,
    COMM_CHANNEL_COUNT
} CommunicationChannel;

/* 对上层暴露的通信接口枚举。 */
typedef enum
{
    COMM_INTERFACE_LOG_UART1 = 0,
    COMM_INTERFACE_RK3576_UART3 = 1,
    COMM_INTERFACE_COUNT
} CommunicationInterfaceId;

typedef ProtocolFrameView CommunicationFrameView;

/* 上层业务协议帧回调。 */
typedef void (*CommunicationProtocolFrameCallback)(void *context,
                                                   CommunicationInterfaceId interface_id,
                                                   const CommunicationFrameView *frame);
/* 日志串口原始数据回调。 */
typedef void (*CommunicationLogRxCallback)(void *context,
                                           CommunicationChannel channel,
                                           const uint8_t *data,
                                           size_t length);

/* 通信模块向上层暴露的回调集合。 */
typedef struct
{
    CommunicationProtocolFrameCallback on_protocol_frame;
    CommunicationLogRxCallback on_log_rx;
    void *context;
} CommunicationCallbacks;

/* 初始化 UART1/UART3 通信资源。 */
void Communication_Init(void);
/* 注册收包回调。 */
void Communication_SetCallbacks(const CommunicationCallbacks *callbacks);
/* 推进底层串口 DMA/IDLE 收包轮询。 */
void Communication_PollRx(void);

/* 发送指定接口的一帧原始协议数据。 */
uint8_t Communication_SendFrame(CommunicationInterfaceId interface_id,
                                uint16_t frame_id,
                                uint8_t data_type,
                                const void *payload,
                                uint16_t payload_length);
/* 发送各基础类型的便捷封装。 */
uint8_t Communication_SendU8(uint16_t frame_id, uint8_t value);
uint8_t Communication_SendU16(uint16_t frame_id, uint16_t value);
uint8_t Communication_SendU32(uint16_t frame_id, uint32_t value);
uint8_t Communication_SendF32(uint16_t frame_id, float value);
uint8_t Communication_SendText(uint16_t frame_id, const char *text);

/* 向日志串口发送文本。 */
void Communication_WriteLog(const char *text);
/* 向日志串口发送原始字节流。 */
int Communication_WriteLogBuffer(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
