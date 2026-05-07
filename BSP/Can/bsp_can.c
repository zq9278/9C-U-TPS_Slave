#include "BSP/Can/bsp_can.h"

#include <string.h>

/**
 * @brief 解析当前发送应使用的超时时间。
 */
static uint32_t BspCan_ResolveTimeout(const BspCan *bus, uint32_t timeout_ms)
{
    if (timeout_ms != 0U)
    {
        return timeout_ms;
    }

    if ((bus != NULL) && (bus->config.default_timeout_ms != 0U))
    {
        return bus->config.default_timeout_ms;
    }

    return BSP_CAN_DEFAULT_TIMEOUT_MS;
}

/**
 * @brief 检查 CAN 总线对象是否处于可用状态。
 */
static uint8_t BspCan_IsReady(const BspCan *bus)
{
    if ((bus == NULL) || (bus->config.ops.send == NULL))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 初始化 CAN 总线对象。
 */
void BspCan_Init(BspCan *bus, const BspCanConfig *config)
{
    if ((bus == NULL) || (config == NULL))
    {
        return;
    }

    (void)memset(bus, 0, sizeof(*bus));
    bus->config = *config;

    if (bus->config.default_timeout_ms == 0U)
    {
        bus->config.default_timeout_ms = BSP_CAN_DEFAULT_TIMEOUT_MS;
    }
}

/**
 * @brief 注册上层 CAN 事件回调。
 */
void BspCan_SetCallbacks(BspCan *bus, const BspCanCallbacks *callbacks)
{
    if ((bus == NULL) || (callbacks == NULL))
    {
        return;
    }

    bus->callbacks = *callbacks;
}

/**
 * @brief 启动 CAN 总线。
 *
 * 该接口本身不直接依赖 HAL，真正如何配置过滤器、启动控制器、打开中断，
 * 由底层 `start` 回调决定。
 */
uint8_t BspCan_Start(BspCan *bus)
{
    if (bus == NULL)
    {
        return 0U;
    }

    if (bus->config.ops.start == NULL)
    {
        bus->started = 1U;
        return 1U;
    }

    bus->started = bus->config.ops.start(bus->config.low_level_context);
    return bus->started;
}

/**
 * @brief 发送一帧 CAN 数据。
 */
uint8_t BspCan_Send(BspCan *bus, const BspCanFrame *frame, uint32_t timeout_ms)
{
    if ((BspCan_IsReady(bus) == 0U) || (frame == NULL))
    {
        return 0U;
    }

    if ((bus->started == 0U) && (BspCan_Start(bus) == 0U))
    {
        return 0U;
    }

    if (bus->config.ops.send(bus->config.low_level_context,
                             frame,
                             BspCan_ResolveTimeout(bus, timeout_ms)) == 0U)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 在接收中断上下文中把一帧 CAN 数据上抛给上层。
 */
void BspCan_OnRxFrameFromIsr(BspCan *bus, const BspCanFrame *frame)
{
    if ((bus == NULL) || (frame == NULL))
    {
        return;
    }

    bus->rx_count++;

    if (bus->callbacks.on_rx_frame != NULL)
    {
        bus->callbacks.on_rx_frame(bus->callbacks.context, frame);
    }
}

/**
 * @brief 在发送完成中断上下文中通知上层。
 */
void BspCan_OnTxCompleteFromIsr(BspCan *bus)
{
    if (bus == NULL)
    {
        return;
    }

    bus->tx_count++;

    if (bus->callbacks.on_tx_complete != NULL)
    {
        bus->callbacks.on_tx_complete(bus->callbacks.context);
    }
}

/**
 * @brief 在错误中断上下文中记录错误并尝试恢复。
 */
void BspCan_OnErrorFromIsr(BspCan *bus, uint32_t error_code)
{
    if (bus == NULL)
    {
        return;
    }

    bus->error_count++;

    if (bus->callbacks.on_error != NULL)
    {
        bus->callbacks.on_error(bus->callbacks.context, error_code);
    }

    if ((bus->config.auto_recover_on_error != 0U) &&
        (bus->config.ops.recover != NULL))
    {
        (void)bus->config.ops.recover(bus->config.low_level_context);
    }
}
