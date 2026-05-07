#include "BSP/I2c/bsp_i2c.h"

#include <string.h>

/**
 * @brief 解析本次读写应使用的超时时间。
 */
static uint32_t BspI2c_ResolveTimeout(const BspI2c *bus, uint32_t timeout_ms)
{
    if (timeout_ms != 0U)
    {
        return timeout_ms;
    }

    if ((bus != NULL) && (bus->config.default_timeout_ms != 0U))
    {
        return bus->config.default_timeout_ms;
    }

    return BSP_I2C_DEFAULT_TIMEOUT_MS;
}

/**
 * @brief 检查总线对象及其底层回调是否准备就绪。
 */
static uint8_t BspI2c_IsReady(const BspI2c *bus)
{
    if ((bus == NULL) ||
        (bus->config.ops.read_mem == NULL) ||
        (bus->config.ops.write_mem == NULL))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 统一底层读调用入口。
 */
static uint8_t BspI2c_MemRead(BspI2c *bus,
                              uint8_t dev_addr,
                              uint8_t reg_addr,
                              uint8_t *data,
                              uint16_t size,
                              uint32_t timeout_ms)
{
    if ((BspI2c_IsReady(bus) == 0U) || (data == NULL) || (size == 0U))
    {
        return 0U;
    }

    return bus->config.ops.read_mem(bus->config.low_level_context,
                                    dev_addr,
                                    reg_addr,
                                    data,
                                    size,
                                    BspI2c_ResolveTimeout(bus, timeout_ms));
}

/**
 * @brief 统一底层写调用入口。
 */
static uint8_t BspI2c_MemWrite(BspI2c *bus,
                               uint8_t dev_addr,
                               uint8_t reg_addr,
                               const uint8_t *data,
                               uint16_t size,
                               uint32_t timeout_ms)
{
    if ((BspI2c_IsReady(bus) == 0U) || (data == NULL) || (size == 0U))
    {
        return 0U;
    }

    return bus->config.ops.write_mem(bus->config.low_level_context,
                                     dev_addr,
                                     reg_addr,
                                     data,
                                     size,
                                     BspI2c_ResolveTimeout(bus, timeout_ms));
}

void BspI2c_Init(BspI2c *bus, const BspI2cConfig *config)
{
    if ((bus == NULL) || (config == NULL))
    {
        return;
    }

    (void)memset(bus, 0, sizeof(*bus));
    bus->config = *config;

    if (bus->config.default_timeout_ms == 0U)
    {
        bus->config.default_timeout_ms = BSP_I2C_DEFAULT_TIMEOUT_MS;
    }
}

void BspI2c_SetDefaultTimeout(BspI2c *bus, uint32_t timeout_ms)
{
    if (bus == NULL)
    {
        return;
    }

    bus->config.default_timeout_ms = timeout_ms;
}

uint8_t BspI2c_ReadBit(BspI2c *bus,
                       uint8_t dev_addr,
                       uint8_t reg_addr,
                       uint8_t bit_num,
                       uint8_t *data,
                       uint32_t timeout_ms)
{
    uint8_t byte_value = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    if (BspI2c_ReadByte(bus, dev_addr, reg_addr, &byte_value, timeout_ms) == 0U)
    {
        return 0U;
    }

    *data = (uint8_t)(byte_value & (uint8_t)(1U << bit_num));
    return 1U;
}

uint8_t BspI2c_ReadBitW(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t bit_num,
                        uint16_t *data,
                        uint32_t timeout_ms)
{
    uint16_t word_value = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    if (BspI2c_ReadWord(bus, dev_addr, reg_addr, &word_value, timeout_ms) == 0U)
    {
        return 0U;
    }

    *data = (uint16_t)(word_value & (uint16_t)(1U << bit_num));
    return 1U;
}

uint8_t BspI2c_ReadBits(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t bit_start,
                        uint8_t length,
                        uint8_t *data,
                        uint32_t timeout_ms)
{
    uint8_t byte_value = 0U;
    uint8_t mask;

    if ((data == NULL) || (length == 0U) || (length > 8U))
    {
        return 0U;
    }

    if (BspI2c_ReadByte(bus, dev_addr, reg_addr, &byte_value, timeout_ms) == 0U)
    {
        return 0U;
    }

    mask = (uint8_t)(((1U << length) - 1U) << (bit_start - length + 1U));
    byte_value &= mask;
    byte_value = (uint8_t)(byte_value >> (bit_start - length + 1U));
    *data = byte_value;
    return 1U;
}

uint8_t BspI2c_ReadBitsW(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t bit_start,
                         uint8_t length,
                         uint16_t *data,
                         uint32_t timeout_ms)
{
    uint16_t word_value = 0U;
    uint16_t mask;

    if ((data == NULL) || (length == 0U) || (length > 16U))
    {
        return 0U;
    }

    if (BspI2c_ReadWord(bus, dev_addr, reg_addr, &word_value, timeout_ms) == 0U)
    {
        return 0U;
    }

    mask = (uint16_t)(((1U << length) - 1U) << (bit_start - length + 1U));
    word_value &= mask;
    word_value = (uint16_t)(word_value >> (bit_start - length + 1U));
    *data = word_value;
    return 1U;
}

uint8_t BspI2c_ReadByte(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t *data,
                        uint32_t timeout_ms)
{
    return BspI2c_ReadBytes(bus, dev_addr, reg_addr, 1U, data, timeout_ms);
}

uint8_t BspI2c_ReadWord(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint16_t *data,
                        uint32_t timeout_ms)
{
    return BspI2c_ReadWords(bus, dev_addr, reg_addr, 1U, data, timeout_ms);
}

uint8_t BspI2c_ReadBytes(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t length,
                         uint8_t *data,
                         uint32_t timeout_ms)
{
    if (BspI2c_MemRead(bus, dev_addr, reg_addr, data, length, timeout_ms) == 0U)
    {
        return 0U;
    }

    return length;
}

uint8_t BspI2c_ReadWords(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t length,
                         uint16_t *data,
                         uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    if (BspI2c_MemRead(bus,
                       dev_addr,
                       reg_addr,
                       (uint8_t *)data,
                       (uint16_t)(length * 2U),
                       timeout_ms) == 0U)
    {
        return 0U;
    }

    return length;
}

uint8_t BspI2c_WriteBit(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t bit_num,
                        uint8_t data)
{
    uint8_t byte_value = 0U;

    if (BspI2c_ReadByte(bus, dev_addr, reg_addr, &byte_value, 0U) == 0U)
    {
        return 0U;
    }

    byte_value = (data != 0U)
                     ? (uint8_t)(byte_value | (uint8_t)(1U << bit_num))
                     : (uint8_t)(byte_value & (uint8_t)~(1U << bit_num));

    return BspI2c_WriteByte(bus, dev_addr, reg_addr, byte_value);
}

uint8_t BspI2c_WriteBitW(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t bit_num,
                         uint16_t data)
{
    uint16_t word_value = 0U;

    if (BspI2c_ReadWord(bus, dev_addr, reg_addr, &word_value, 0U) == 0U)
    {
        return 0U;
    }

    word_value = (data != 0U)
                     ? (uint16_t)(word_value | (uint16_t)(1U << bit_num))
                     : (uint16_t)(word_value & (uint16_t)~(1U << bit_num));

    return BspI2c_WriteWord(bus, dev_addr, reg_addr, word_value);
}

uint8_t BspI2c_WriteBits(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t bit_start,
                         uint8_t length,
                         uint8_t data)
{
    uint8_t byte_value = 0U;
    uint8_t mask;

    if ((length == 0U) || (length > 8U))
    {
        return 0U;
    }

    if (BspI2c_ReadByte(bus, dev_addr, reg_addr, &byte_value, 0U) == 0U)
    {
        return 0U;
    }

    mask = (uint8_t)(((1U << length) - 1U) << (bit_start - length + 1U));
    data = (uint8_t)(data << (bit_start - length + 1U));
    data &= mask;
    byte_value &= (uint8_t)(~mask);
    byte_value |= data;

    return BspI2c_WriteByte(bus, dev_addr, reg_addr, byte_value);
}

uint8_t BspI2c_WriteBitsW(BspI2c *bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t bit_start,
                          uint8_t length,
                          uint16_t data)
{
    uint16_t word_value = 0U;
    uint16_t mask;

    if ((length == 0U) || (length > 16U))
    {
        return 0U;
    }

    if (BspI2c_ReadWord(bus, dev_addr, reg_addr, &word_value, 0U) == 0U)
    {
        return 0U;
    }

    mask = (uint16_t)(((1U << length) - 1U) << (bit_start - length + 1U));
    data = (uint16_t)(data << (bit_start - length + 1U));
    data &= mask;
    word_value &= (uint16_t)(~mask);
    word_value |= data;

    return BspI2c_WriteWord(bus, dev_addr, reg_addr, word_value);
}

uint8_t BspI2c_WriteByte(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t data)
{
    return BspI2c_WriteBytes(bus, dev_addr, reg_addr, 1U, &data);
}

uint8_t BspI2c_WriteWord(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint16_t data)
{
    return BspI2c_WriteWords(bus, dev_addr, reg_addr, 1U, &data);
}

uint8_t BspI2c_WriteBytes(BspI2c *bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t length,
                          const uint8_t *data)
{
    return BspI2c_MemWrite(bus, dev_addr, reg_addr, data, length, 0U);
}

uint8_t BspI2c_WriteWords(BspI2c *bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t length,
                          const uint16_t *data)
{
    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    return BspI2c_MemWrite(bus,
                           dev_addr,
                           reg_addr,
                           (const uint8_t *)data,
                           (uint16_t)(length * 2U),
                           0U);
}

uint8_t BspI2c_MasterTransmit(BspI2c *bus,
                              uint8_t dev_addr,
                              const uint8_t *data,
                              uint16_t size,
                              uint32_t timeout_ms)
{
    if ((bus == NULL) ||
        (bus->config.ops.master_transmit == NULL) ||
        (data == NULL) ||
        (size == 0U))
    {
        return 0U;
    }

    return bus->config.ops.master_transmit(bus->config.low_level_context,
                                           dev_addr,
                                           data,
                                           size,
                                           BspI2c_ResolveTimeout(bus, timeout_ms));
}

uint8_t BspI2c_MasterReceive(BspI2c *bus,
                             uint8_t dev_addr,
                             uint8_t *data,
                             uint16_t size,
                             uint32_t timeout_ms)
{
    if ((bus == NULL) ||
        (bus->config.ops.master_receive == NULL) ||
        (data == NULL) ||
        (size == 0U))
    {
        return 0U;
    }

    return bus->config.ops.master_receive(bus->config.low_level_context,
                                          dev_addr,
                                          data,
                                          size,
                                          BspI2c_ResolveTimeout(bus, timeout_ms));
}

void BspI2c_Recover(BspI2c *bus)
{
    if ((bus == NULL) || (bus->config.ops.recover == NULL))
    {
        return;
    }

    bus->config.ops.recover(bus->config.low_level_context);
}
