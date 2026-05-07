#include "bsp_i2c_soft_adapter.h"

#include <stddef.h>
#include <string.h>

static void SoftDelay(BspI2cSoftAdapter *adapter)
{
    if ((adapter != NULL) && (adapter->delay != NULL))
    {
        adapter->delay(adapter->line_context);
    }
}

static void SoftScl(BspI2cSoftAdapter *adapter, uint8_t level)
{
    if ((adapter != NULL) && (adapter->write_scl != NULL))
    {
        adapter->write_scl(adapter->line_context, level);
    }
}

static void SoftSda(BspI2cSoftAdapter *adapter, uint8_t level)
{
    if ((adapter != NULL) && (adapter->write_sda != NULL))
    {
        adapter->write_sda(adapter->line_context, level);
    }
}

static uint8_t SoftReadSda(BspI2cSoftAdapter *adapter)
{
    if ((adapter == NULL) || (adapter->read_sda == NULL))
    {
        return 1U;
    }

    return adapter->read_sda(adapter->line_context);
}

static void SoftStart(BspI2cSoftAdapter *adapter)
{
    SoftSda(adapter, 1U); SoftScl(adapter, 1U); SoftDelay(adapter);
    SoftSda(adapter, 0U); SoftDelay(adapter);
    SoftScl(adapter, 0U); SoftDelay(adapter);
}

static void SoftStop(BspI2cSoftAdapter *adapter)
{
    SoftSda(adapter, 0U); SoftScl(adapter, 1U); SoftDelay(adapter);
    SoftSda(adapter, 1U); SoftDelay(adapter);
}

static uint8_t SoftWriteByte(BspI2cSoftAdapter *adapter, uint8_t value)
{
    uint8_t bit;
    uint8_t ack;

    for (bit = 0U; bit < 8U; ++bit)
    {
        SoftSda(adapter, (uint8_t)((value & 0x80U) != 0U));
        SoftDelay(adapter);
        SoftScl(adapter, 1U); SoftDelay(adapter);
        SoftScl(adapter, 0U); SoftDelay(adapter);
        value <<= 1U;
    }

    SoftSda(adapter, 1U);
    SoftDelay(adapter);
    SoftScl(adapter, 1U);
    SoftDelay(adapter);
    ack = (uint8_t)(SoftReadSda(adapter) == 0U);
    SoftScl(adapter, 0U);
    SoftDelay(adapter);

    return ack;
}

static uint8_t SoftReadByte(BspI2cSoftAdapter *adapter, uint8_t ack)
{
    uint8_t bit;
    uint8_t value = 0U;

    SoftSda(adapter, 1U);
    for (bit = 0U; bit < 8U; ++bit)
    {
        value <<= 1U;
        SoftScl(adapter, 1U);
        SoftDelay(adapter);
        if (SoftReadSda(adapter) != 0U)
        {
            value |= 1U;
        }
        SoftScl(adapter, 0U);
        SoftDelay(adapter);
    }

    SoftSda(adapter, (uint8_t)(ack == 0U));
    SoftDelay(adapter);
    SoftScl(adapter, 1U); SoftDelay(adapter);
    SoftScl(adapter, 0U); SoftDelay(adapter);
    SoftSda(adapter, 1U);

    return value;
}

static uint8_t SoftWriteMem(void *context,
                            uint8_t dev_addr,
                            uint8_t reg_addr,
                            const uint8_t *data,
                            uint16_t size,
                            uint32_t timeout_ms)
{
    BspI2cSoftAdapter *adapter = (BspI2cSoftAdapter *)context;
    uint16_t index;
    (void)timeout_ms;

    if ((adapter == NULL) || ((size > 0U) && (data == NULL)))
    {
        return 0U;
    }

    SoftStart(adapter);
    if ((SoftWriteByte(adapter, (uint8_t)(dev_addr << 1U)) == 0U) ||
        (SoftWriteByte(adapter, reg_addr) == 0U))
    {
        SoftStop(adapter);
        return 0U;
    }

    for (index = 0U; index < size; ++index)
    {
        if (SoftWriteByte(adapter, data[index]) == 0U)
        {
            SoftStop(adapter);
            return 0U;
        }
    }

    SoftStop(adapter);
    return 1U;
}

static uint8_t SoftReadMem(void *context,
                           uint8_t dev_addr,
                           uint8_t reg_addr,
                           uint8_t *data,
                           uint16_t size,
                           uint32_t timeout_ms)
{
    BspI2cSoftAdapter *adapter = (BspI2cSoftAdapter *)context;
    uint16_t index;
    (void)timeout_ms;

    if ((adapter == NULL) || ((size > 0U) && (data == NULL)))
    {
        return 0U;
    }

    SoftStart(adapter);
    if ((SoftWriteByte(adapter, (uint8_t)(dev_addr << 1U)) == 0U) ||
        (SoftWriteByte(adapter, reg_addr) == 0U))
    {
        SoftStop(adapter);
        return 0U;
    }

    SoftStart(adapter);
    if (SoftWriteByte(adapter, (uint8_t)((dev_addr << 1U) | 1U)) == 0U)
    {
        SoftStop(adapter);
        return 0U;
    }

    for (index = 0U; index < size; ++index)
    {
        data[index] = SoftReadByte(adapter, (uint8_t)(index + 1U < size));
    }

    SoftStop(adapter);
    return 1U;
}

void BspI2cSoftAdapter_MakeConfig(BspI2cConfig *config,
                                  BspI2cSoftAdapter *adapter,
                                  uint32_t default_timeout_ms)
{
    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    config->ops.read_mem = SoftReadMem;
    config->ops.write_mem = SoftWriteMem;
    config->low_level_context = adapter;
    config->default_timeout_ms = default_timeout_ms;
}
