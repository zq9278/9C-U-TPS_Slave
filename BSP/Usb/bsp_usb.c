#include "BSP/Usb/bsp_usb.h"

#include <string.h>

/**
 * @brief 检查 USB 总线对象是否可用。
 */
static uint8_t BspUsb_IsReady(const BspUsb *usb)
{
    if ((usb == NULL) || (usb->config.ops.send_packet == NULL))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 初始化 USB 总线对象。
 */
void BspUsb_Init(BspUsb *usb, const BspUsbConfig *config)
{
    if ((usb == NULL) || (config == NULL))
    {
        return;
    }

    (void)memset(usb, 0, sizeof(*usb));
    usb->config = *config;
}

/**
 * @brief 注册上层 USB 事件回调。
 */
void BspUsb_SetCallbacks(BspUsb *usb, const BspUsbCallbacks *callbacks)
{
    if ((usb == NULL) || (callbacks == NULL))
    {
        return;
    }

    usb->callbacks = *callbacks;
}

/**
 * @brief 启动 USB 链路。
 */
uint8_t BspUsb_Start(BspUsb *usb)
{
    if (usb == NULL)
    {
        return 0U;
    }

    if (usb->config.ops.start == NULL)
    {
        usb->started = 1U;
        return 1U;
    }

    usb->started = usb->config.ops.start(usb->config.low_level_context);
    return usb->started;
}

/**
 * @brief 通过指定 endpoint 发送一包 USB 数据。
 */
uint8_t BspUsb_SendPacket(BspUsb *usb,
                          uint8_t endpoint,
                          const uint8_t *data,
                          uint16_t length)
{
    if ((BspUsb_IsReady(usb) == 0U) ||
        (data == NULL) ||
        (length == 0U))
    {
        return 0U;
    }

    if ((usb->started == 0U) && (BspUsb_Start(usb) == 0U))
    {
        return 0U;
    }

    return usb->config.ops.send_packet(usb->config.low_level_context,
                                       endpoint,
                                       data,
                                       length);
}

/**
 * @brief 在任务上下文中处理 ISR 缓存下来的 USB 包。
 *
 * 这一步的意义和你原来 `interface_usb.cpp` 里的 deferred task 很像：
 * 把 ISR 中收包和上层协议解析解耦开，避免在中断里做太重的工作。
 */
void BspUsb_Poll(BspUsb *usb)
{
    while ((usb != NULL) && (usb->rx_count > 0U))
    {
        BspUsbPacket packet;

        packet = usb->rx_queue[usb->rx_tail];
        usb->rx_tail = (uint8_t)((usb->rx_tail + 1U) % BSP_USB_RX_QUEUE_DEPTH);
        usb->rx_count--;

        if (usb->callbacks.on_packet_received != NULL)
        {
            usb->callbacks.on_packet_received(usb->callbacks.context, &packet);
        }

        if ((usb->config.auto_rearm_rx != 0U) &&
            (usb->config.ops.rearm_rx != NULL))
        {
            (void)usb->config.ops.rearm_rx(usb->config.low_level_context, packet.endpoint);
        }
    }
}

/**
 * @brief 在接收中断上下文中缓存一包新收到的 USB 数据。
 */
void BspUsb_OnPacketReceivedFromIsr(BspUsb *usb,
                                    uint8_t endpoint,
                                    const uint8_t *data,
                                    uint16_t length)
{
    BspUsbPacket *slot;

    if ((usb == NULL) || (data == NULL) || (length == 0U))
    {
        return;
    }

    if (length > BSP_USB_MAX_PACKET_SIZE)
    {
        length = BSP_USB_MAX_PACKET_SIZE;
    }

    if (usb->rx_count >= BSP_USB_RX_QUEUE_DEPTH)
    {
        usb->dropped_packets++;
        return;
    }

    slot = &usb->rx_queue[usb->rx_head];
    slot->endpoint = endpoint;
    slot->length = length;
    (void)memcpy(slot->data, data, length);

    usb->rx_head = (uint8_t)((usb->rx_head + 1U) % BSP_USB_RX_QUEUE_DEPTH);
    usb->rx_count++;
}

/**
 * @brief 在发送完成中断上下文中通知上层。
 */
void BspUsb_OnTxCompleteFromIsr(BspUsb *usb, uint8_t endpoint)
{
    if ((usb == NULL) || (usb->callbacks.on_tx_complete == NULL))
    {
        return;
    }

    usb->callbacks.on_tx_complete(usb->callbacks.context, endpoint);
}

/**
 * @brief 在错误中断上下文中把 USB 错误上抛给上层。
 */
void BspUsb_OnErrorFromIsr(BspUsb *usb, uint32_t error_code)
{
    if ((usb == NULL) || (usb->callbacks.on_error == NULL))
    {
        return;
    }

    usb->callbacks.on_error(usb->callbacks.context, error_code);
}
