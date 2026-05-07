#include "BSP/Spi/bsp_spi.h"

#include <string.h>

/**
 * @brief 解析本次传输应使用的超时时间。
 */
static uint32_t BspSpi_ResolveTimeout(const BspSpi *bus, uint32_t timeout_ms)
{
    if (timeout_ms != 0U)
    {
        return timeout_ms;
    }

    if ((bus != NULL) && (bus->config.default_timeout_ms != 0U))
    {
        return bus->config.default_timeout_ms;
    }

    return BSP_SPI_DEFAULT_TIMEOUT_MS;
}

/**
 * @brief 检查 SPI 总线对象是否可用。
 */
static uint8_t BspSpi_IsReady(const BspSpi *bus)
{
    if (bus == NULL)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 如果启用了自动片选，则在传输开始前拉低片选。
 */
static void BspSpi_BeginTransfer(BspSpi *bus)
{
    if ((bus == NULL) ||
        (bus->config.auto_chip_select == 0U) ||
        (bus->config.ops.chip_select == NULL))
    {
        return;
    }

    bus->config.ops.chip_select(bus->config.chip_select_context, 1U);
}

/**
 * @brief 如果启用了自动片选，则在传输结束后释放片选。
 */
static void BspSpi_EndTransfer(BspSpi *bus)
{
    if ((bus == NULL) ||
        (bus->config.auto_chip_select == 0U) ||
        (bus->config.ops.chip_select == NULL))
    {
        return;
    }

    bus->config.ops.chip_select(bus->config.chip_select_context, 0U);
}

/**
 * @brief 初始化 SPI 总线对象。
 */
void BspSpi_Init(BspSpi *bus, const BspSpiConfig *config)
{
    if ((bus == NULL) || (config == NULL))
    {
        return;
    }

    (void)memset(bus, 0, sizeof(*bus));
    bus->config = *config;

    if (bus->config.default_timeout_ms == 0U)
    {
        bus->config.default_timeout_ms = BSP_SPI_DEFAULT_TIMEOUT_MS;
    }
}

/**
 * @brief 修改总线对象的默认超时时间。
 */
void BspSpi_SetDefaultTimeout(BspSpi *bus, uint32_t timeout_ms)
{
    if (bus == NULL)
    {
        return;
    }

    bus->config.default_timeout_ms = timeout_ms;
}

/**
 * @brief 手动拉低片选。
 *
 * 当某些设备需要一次事务内执行多次 SPI 传输时，可以关闭自动片选，
 * 然后用 `BspSpi_Select()` / `BspSpi_Deselect()` 手动包裹整个事务。
 */
void BspSpi_Select(BspSpi *bus)
{
    if ((bus == NULL) || (bus->config.ops.chip_select == NULL))
    {
        return;
    }

    bus->config.ops.chip_select(bus->config.chip_select_context, 1U);
}

/**
 * @brief 手动释放片选。
 */
void BspSpi_Deselect(BspSpi *bus)
{
    if ((bus == NULL) || (bus->config.ops.chip_select == NULL))
    {
        return;
    }

    bus->config.ops.chip_select(bus->config.chip_select_context, 0U);
}

/**
 * @brief 发一段纯发送 SPI 数据。
 */
uint8_t BspSpi_Transmit(BspSpi *bus,
                        const uint8_t *data,
                        uint16_t size,
                        uint32_t timeout_ms)
{
    uint8_t ok;

    if ((BspSpi_IsReady(bus) == 0U) ||
        (bus->config.ops.transmit == NULL) ||
        (data == NULL) ||
        (size == 0U))
    {
        return 0U;
    }

    BspSpi_BeginTransfer(bus);
    ok = bus->config.ops.transmit(bus->config.low_level_context,
                                  data,
                                  size,
                                  BspSpi_ResolveTimeout(bus, timeout_ms));
    BspSpi_EndTransfer(bus);
    return ok;
}

/**
 * @brief 发 dummy 数据并接收一段 SPI 数据。
 */
uint8_t BspSpi_Receive(BspSpi *bus,
                       uint8_t *data,
                       uint16_t size,
                       uint32_t timeout_ms)
{
    uint8_t ok;

    if ((BspSpi_IsReady(bus) == 0U) ||
        (bus->config.ops.receive == NULL) ||
        (data == NULL) ||
        (size == 0U))
    {
        return 0U;
    }

    BspSpi_BeginTransfer(bus);
    ok = bus->config.ops.receive(bus->config.low_level_context,
                                 data,
                                 size,
                                 BspSpi_ResolveTimeout(bus, timeout_ms));
    BspSpi_EndTransfer(bus);
    return ok;
}

/**
 * @brief 执行一次标准 SPI 全双工收发。
 */
uint8_t BspSpi_TransmitReceive(BspSpi *bus,
                               const uint8_t *tx_data,
                               uint8_t *rx_data,
                               uint16_t size,
                               uint32_t timeout_ms)
{
    uint8_t ok;

    if ((BspSpi_IsReady(bus) == 0U) ||
        (bus->config.ops.transmit_receive == NULL) ||
        (tx_data == NULL) ||
        (rx_data == NULL) ||
        (size == 0U))
    {
        return 0U;
    }

    BspSpi_BeginTransfer(bus);
    ok = bus->config.ops.transmit_receive(bus->config.low_level_context,
                                          tx_data,
                                          rx_data,
                                          size,
                                          BspSpi_ResolveTimeout(bus, timeout_ms));
    BspSpi_EndTransfer(bus);
    return ok;
}
