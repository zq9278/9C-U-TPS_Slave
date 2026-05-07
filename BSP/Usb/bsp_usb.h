#ifndef BSP_USB_H
#define BSP_USB_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    /**
     * @brief USB 单包最大缓存大小。
     *
     * 当前用于 BSP 内部把 ISR 收到的包先缓存起来，再延后到任务上下文处理。
     */
    BSP_USB_MAX_PACKET_SIZE = 128U,
    BSP_USB_RX_QUEUE_DEPTH = 4U
};

/**
 * @brief 一包 USB 数据的抽象表示。
 */
typedef struct
{
    uint8_t endpoint;
    uint16_t length;
    uint8_t data[BSP_USB_MAX_PACKET_SIZE];
} BspUsbPacket;

/**
 * @brief USB 底层启动回调。
 *
 * 典型职责包括启动 USB Device、使能 CDC 接收等。
 */
typedef uint8_t (*BspUsbStartCallback)(void *context);

/**
 * @brief USB 底层发送回调。
 *
 * 当前按“包”发送，不按字符流发送。
 */
typedef uint8_t (*BspUsbSendPacketCallback)(void *context,
                                            uint8_t endpoint,
                                            const uint8_t *data,
                                            uint16_t length);

/**
 * @brief USB 底层重新 arm 接收回调。
 *
 * 某些 USB Device 栈在每次收到一包后，需要显式调用一次“允许下一包进入”。
 */
typedef uint8_t (*BspUsbRearmRxCallback)(void *context, uint8_t endpoint);

/**
 * @brief USB 底层适配操作集合。
 */
typedef struct
{
    BspUsbStartCallback start;
    BspUsbSendPacketCallback send_packet;
    BspUsbRearmRxCallback rearm_rx;
} BspUsbOps;

/**
 * @brief 上层感兴趣的 USB 事件回调集合。
 */
typedef struct
{
    void (*on_packet_received)(void *context, const BspUsbPacket *packet);
    void (*on_tx_complete)(void *context, uint8_t endpoint);
    void (*on_error)(void *context, uint32_t error_code);
    void *context;
} BspUsbCallbacks;

/**
 * @brief USB 总线初始化配置。
 */
typedef struct
{
    BspUsbOps ops;
    void *low_level_context;
    uint8_t auto_rearm_rx;
} BspUsbConfig;

/**
 * @brief USB 总线对象。
 *
 * 这里更像“USB 通道管理器”而不是传统意义上的物理总线，
 * 因为 USB 接收是按包、按 endpoint 处理的。
 */
typedef struct
{
    BspUsbConfig config;
    BspUsbCallbacks callbacks;
    BspUsbPacket rx_queue[BSP_USB_RX_QUEUE_DEPTH];
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_count;
    uint8_t started;
    uint32_t dropped_packets;
} BspUsb;

void BspUsb_Init(BspUsb *usb, const BspUsbConfig *config);
void BspUsb_SetCallbacks(BspUsb *usb, const BspUsbCallbacks *callbacks);
uint8_t BspUsb_Start(BspUsb *usb);
uint8_t BspUsb_SendPacket(BspUsb *usb,
                          uint8_t endpoint,
                          const uint8_t *data,
                          uint16_t length);
void BspUsb_Poll(BspUsb *usb);
void BspUsb_OnPacketReceivedFromIsr(BspUsb *usb,
                                    uint8_t endpoint,
                                    const uint8_t *data,
                                    uint16_t length);
void BspUsb_OnTxCompleteFromIsr(BspUsb *usb, uint8_t endpoint);
void BspUsb_OnErrorFromIsr(BspUsb *usb, uint32_t error_code);

#ifdef __cplusplus
}
#endif

#endif
