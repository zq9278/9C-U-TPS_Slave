#ifndef USERDRIVERS_EEPROM24CXX_H
#define USERDRIVERS_EEPROM24CXX_H

#include <stdint.h>
#include "BSP/Gpio/bsp_gpio.h"
#include "BSP/I2c/bsp_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USER_EEPROM24CXX_ADDR_7BIT 0x50U

typedef enum
{
    USER_EEPROM24C01 = 127U,
    USER_EEPROM24C02 = 255U,
    USER_EEPROM24C04 = 511U,
    USER_EEPROM24C08 = 1023U,
    USER_EEPROM24C16 = 2047U
} UserEeprom24cxxType;

typedef void (*UserEepromDelayMs)(uint32_t ms);

typedef struct
{
    BspI2c *bus;
    BspGpioPin *write_protect_pin;
    UserEepromDelayMs delay_ms;
    uint16_t last_address;
    uint8_t base_addr_7bit;
    uint8_t page_size;
    uint8_t write_delay_ms;
} UserEeprom24cxx;

void UserEeprom24cxx_Init(UserEeprom24cxx *dev,
                          BspI2c *bus,
                          BspGpioPin *write_protect_pin,
                          UserEeprom24cxxType type,
                          uint8_t page_size,
                          UserEepromDelayMs delay_ms);
uint8_t UserEeprom24cxx_ReadByte(UserEeprom24cxx *dev, uint16_t addr, uint8_t *value);
uint8_t UserEeprom24cxx_WriteByte(UserEeprom24cxx *dev, uint16_t addr, uint8_t value);
uint8_t UserEeprom24cxx_Read(UserEeprom24cxx *dev, uint16_t addr, uint8_t *data, uint16_t size);
uint8_t UserEeprom24cxx_Write(UserEeprom24cxx *dev, uint16_t addr, const uint8_t *data, uint16_t size);
uint8_t UserEeprom24cxx_Check(UserEeprom24cxx *dev);

#ifdef __cplusplus
}
#endif

#endif
