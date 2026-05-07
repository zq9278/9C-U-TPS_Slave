#include "bsp_spi_soft_adapter.h"

#include <stddef.h>

static void SoftDelay(BspSpiSoftAdapter *adapter)
{
    if ((adapter != NULL) && (adapter->delay != NULL))
    {
        adapter->delay(adapter->line_context);
    }
}

static void SoftSck(BspSpiSoftAdapter *adapter, uint8_t level)
{
    if ((adapter != NULL) && (adapter->write_sck != NULL))
    {
        adapter->write_sck(adapter->line_context, level);
    }
}

static void SoftMosi(BspSpiSoftAdapter *adapter, uint8_t level)
{
    if ((adapter != NULL) && (adapter->write_mosi != NULL))
    {
        adapter->write_mosi(adapter->line_context, level);
    }
}

static uint8_t SoftMiso(BspSpiSoftAdapter *adapter)
{
    if ((adapter == NULL) || (adapter->read_miso == NULL))
    {
        return 0U;
    }

    return adapter->read_miso(adapter->line_context);
}

static uint8_t SoftTransferByte(BspSpiSoftAdapter *adapter, uint8_t tx)
{
    uint8_t bit;
    uint8_t rx = 0U;
    uint8_t idle = adapter->cpol ? 1U : 0U;
    uint8_t active = adapter->cpol ? 0U : 1U;

    for (bit = 0U; bit < 8U; ++bit)
    {
        if (adapter->cpha == 0U)
        {
            SoftMosi(adapter, (uint8_t)((tx & 0x80U) != 0U));
            SoftDelay(adapter);
            SoftSck(adapter, active);
            SoftDelay(adapter);
            rx = (uint8_t)((rx << 1U) | (SoftMiso(adapter) != 0U));
            SoftSck(adapter, idle);
        }
        else
        {
            SoftSck(adapter, active);
            SoftMosi(adapter, (uint8_t)((tx & 0x80U) != 0U));
            SoftDelay(adapter);
            SoftSck(adapter, idle);
            SoftDelay(adapter);
            rx = (uint8_t)((rx << 1U) | (SoftMiso(adapter) != 0U));
        }
        tx <<= 1U;
        SoftDelay(adapter);
    }

    return rx;
}

static uint8_t SoftTransmitReceive(void *context,
                                   const uint8_t *tx_data,
                                   uint8_t *rx_data,
                                   uint16_t size,
                                   uint32_t timeout_ms)
{
    BspSpiSoftAdapter *adapter = (BspSpiSoftAdapter *)context;
    uint16_t index;
    (void)timeout_ms;

    if ((adapter == NULL) || ((size > 0U) && ((tx_data == NULL) || (rx_data == NULL))))
    {
        return 0U;
    }

    SoftSck(adapter, adapter->cpol ? 1U : 0U);
    for (index = 0U; index < size; ++index)
    {
        rx_data[index] = SoftTransferByte(adapter, tx_data[index]);
    }

    return 1U;
}

static uint8_t SoftTransmit(void *context,
                            const uint8_t *data,
                            uint16_t size,
                            uint32_t timeout_ms)
{
    uint8_t dummy_rx;
    uint16_t index;
    BspSpiSoftAdapter *adapter = (BspSpiSoftAdapter *)context;
    (void)timeout_ms;

    if ((adapter == NULL) || ((size > 0U) && (data == NULL)))
    {
        return 0U;
    }

    for (index = 0U; index < size; ++index)
    {
        dummy_rx = SoftTransferByte(adapter, data[index]);
        (void)dummy_rx;
    }

    return 1U;
}

static uint8_t SoftReceive(void *context,
                           uint8_t *data,
                           uint16_t size,
                           uint32_t timeout_ms)
{
    uint16_t index;
    BspSpiSoftAdapter *adapter = (BspSpiSoftAdapter *)context;
    (void)timeout_ms;

    if ((adapter == NULL) || ((size > 0U) && (data == NULL)))
    {
        return 0U;
    }

    for (index = 0U; index < size; ++index)
    {
        data[index] = SoftTransferByte(adapter, 0xFFU);
    }

    return 1U;
}

void BspSpiSoftAdapter_MakeConfig(BspSpiConfig *config,
                                  BspSpiSoftAdapter *adapter,
                                  BspSpiChipSelectCallback chip_select,
                                  void *chip_select_context,
                                  uint8_t auto_chip_select,
                                  uint32_t default_timeout_ms)
{
    if (config == NULL)
    {
        return;
    }

    config->ops.transmit = SoftTransmit;
    config->ops.receive = SoftReceive;
    config->ops.transmit_receive = SoftTransmitReceive;
    config->ops.chip_select = chip_select;
    config->low_level_context = adapter;
    config->chip_select_context = chip_select_context;
    config->auto_chip_select = auto_chip_select;
    config->default_timeout_ms = default_timeout_ms;
}
