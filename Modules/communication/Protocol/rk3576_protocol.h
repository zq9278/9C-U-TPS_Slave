#ifndef MODULES_PROTOCOL_RK3576_PROTOCOL_H
#define MODULES_PROTOCOL_RK3576_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "App/System/system_app.h"
#include "Modules/communication/Protocol/protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * RK3576 协议对象封装了两类能力：
 * 1. 主动发包：借助 ProtocolTransport 把协议帧送到底层 UART；
 * 2. 被动收包：利用 ProtocolRxState 在字节流中识别完整帧。
 */

/**
 * @brief RK3576 协议对象。
 *
 * 它同时管理：
 * 1. 底层发送通道；
 * 2. 流式收包状态机；
 * 3. 协议运行时状态；
 * 4. 上层有效帧通知回调。
 */
typedef struct
{
    ProtocolTransport transport;
    ProtocolRxState rx_state;
    void (*on_valid_frame)(void *context, const ProtocolFrameView *frame);
    void *on_valid_frame_context;
} Rk3576Protocol;

/**
 * @brief 初始化 RK3576 协议对象。
 *
 * @param protocol      待初始化的协议对象。
 * @param send_callback 底层发送字节流回调。
 * @param send_context  底层发送回调上下文。
 */
void Rk3576Protocol_Init(Rk3576Protocol *protocol,
                         ProtocolSendBytesCallback send_callback,
                         void *send_context);

/**
 * @brief 设置“收到有效帧”通知回调。
 *
 * @param protocol 协议对象。
 * @param callback 有效帧回调。
 * @param context  回调上下文。
 */
void Rk3576Protocol_SetFrameCallback(Rk3576Protocol *protocol,
                                     void (*callback)(void *context, const ProtocolFrameView *frame),
                                     void *context);

/**
 * @brief 向协议解析器输入一段来自底层串口的新字节流。
 *
 * @param protocol 协议对象。
 * @param data     新收到的字节流。
 * @param length   新收到的字节数。
 */
void Rk3576Protocol_Input(Rk3576Protocol *protocol, const uint8_t *data, size_t length);

/**
 * @brief 主动发送一帧 RK3576 协议。
 *
 * @param protocol       协议对象。
 * @param frame_id       帧 ID。
 * @param data_type      数据类型。
 * @param payload        payload 首地址。
 * @param payload_length payload 长度。
 * @return `1U` 表示发送请求成功提交，`0U` 表示参数错误或底层发送失败。
 */
uint8_t Rk3576Protocol_SendFrame(Rk3576Protocol *protocol,
                                 uint16_t frame_id,
                                 uint8_t data_type,
                                 const uint8_t *payload,
                                 uint16_t payload_length);

uint8_t Rk3576Protocol_FrameToAppCommand(const ProtocolFrameView *frame,
                                         app_cmd_t *command);

#ifdef __cplusplus
}
#endif

#endif
