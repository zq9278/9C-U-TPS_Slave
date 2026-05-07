#include "eeprom24cxx.h"

#include <stddef.h>

static void EepromDelay(UserEeprom24cxx *dev, uint32_t ms)
{
    if ((dev != NULL) && (dev->delay_ms != NULL))
    {
        dev->delay_ms(ms);
    }
}

static void EepromWriteProtect(UserEeprom24cxx *dev, uint8_t enabled)
{
    if (dev->write_protect_pin != NULL)
    {
        BspGpio_WriteActive(dev->write_protect_pin, enabled);
    }
}

static uint8_t EepromDeviceAddr(UserEeprom24cxx *dev, uint16_t mem_addr)
{
    return (uint8_t)(dev->base_addr_7bit | ((mem_addr >> 8U) & 0x07U));
}

void UserEeprom24cxx_Init(UserEeprom24cxx *dev,
                          BspI2c *bus,
                          BspGpioPin *write_protect_pin,
                          UserEeprom24cxxType type,
                          uint8_t page_size,
                          UserEepromDelayMs delay_ms)
{
    if (dev == NULL)
    {
        return;
    }

    dev->bus = bus;
    dev->write_protect_pin = write_protect_pin;
    dev->delay_ms = delay_ms;
    dev->last_address = (uint16_t)type;
    dev->base_addr_7bit = USER_EEPROM24CXX_ADDR_7BIT;
    dev->page_size = (page_size == 0U) ? 8U : page_size;
    dev->write_delay_ms = 10U;

    EepromWriteProtect(dev, 1U);
}

uint8_t UserEeprom24cxx_ReadByte(UserEeprom24cxx *dev, uint16_t addr, uint8_t *value)
{
    if ((dev == NULL) || (dev->bus == NULL) || (value == NULL) || (addr > dev->last_address))
    {
        return 0U;
    }

    return BspI2c_ReadByte(dev->bus, EepromDeviceAddr(dev, addr), (uint8_t)addr, value, 0U);
}

uint8_t UserEeprom24cxx_WriteByte(UserEeprom24cxx *dev, uint16_t addr, uint8_t value)
{
    uint8_t ok;

    if ((dev == NULL) || (dev->bus == NULL) || (addr > dev->last_address))
    {
        return 0U;
    }

    EepromWriteProtect(dev, 0U);
    ok = BspI2c_WriteByte(dev->bus, EepromDeviceAddr(dev, addr), (uint8_t)addr, value);
    EepromDelay(dev, dev->write_delay_ms);
    EepromWriteProtect(dev, 1U);

    return ok;
}

uint8_t UserEeprom24cxx_Read(UserEeprom24cxx *dev, uint16_t addr, uint8_t *data, uint16_t size)
{
    uint16_t offset;

    if ((dev == NULL) || (data == NULL))
    {
        return 0U;
    }

    for (offset = 0U; offset < size; ++offset)
    {
        if (UserEeprom24cxx_ReadByte(dev, (uint16_t)(addr + offset), &data[offset]) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t UserEeprom24cxx_Write(UserEeprom24cxx *dev, uint16_t addr, const uint8_t *data, uint16_t size)
{
    uint16_t written = 0U;

    if ((dev == NULL) || (dev->bus == NULL) || (data == NULL))
    {
        return 0U;
    }

    while (written < size)
    {
        uint16_t cur_addr = (uint16_t)(addr + written);
        uint8_t page_left = (uint8_t)(dev->page_size - (cur_addr % dev->page_size));
        uint16_t chunk = (uint16_t)(size - written);
        uint8_t ok;

        if (cur_addr > dev->last_address)
        {
            return 0U;
        }
        if (chunk > page_left)
        {
            chunk = page_left;
        }
        if ((cur_addr + chunk - 1U) > dev->last_address)
        {
            chunk = (uint16_t)(dev->last_address - cur_addr + 1U);
        }

        EepromWriteProtect(dev, 0U);
        ok = BspI2c_WriteBytes(dev->bus,
                               EepromDeviceAddr(dev, cur_addr),
                               (uint8_t)cur_addr,
                               (uint8_t)chunk,
                               &data[written]);
        EepromDelay(dev, dev->write_delay_ms);
        EepromWriteProtect(dev, 1U);
        if (ok == 0U)
        {
            return 0U;
        }

        written = (uint16_t)(written + chunk);
    }

    return 1U;
}

uint8_t UserEeprom24cxx_Check(UserEeprom24cxx *dev)
{
    uint8_t value = 0U;

    if (UserEeprom24cxx_ReadByte(dev, dev->last_address, &value) == 0U)
    {
        return 0U;
    }
    if (value == 0x55U)
    {
        return 1U;
    }
    if (UserEeprom24cxx_WriteByte(dev, dev->last_address, 0x55U) == 0U)
    {
        return 0U;
    }
    if (UserEeprom24cxx_ReadByte(dev, dev->last_address, &value) == 0U)
    {
        return 0U;
    }

    return (uint8_t)(value == 0x55U);
}
