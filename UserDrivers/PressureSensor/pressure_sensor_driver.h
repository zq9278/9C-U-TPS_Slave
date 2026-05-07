#ifndef USERDRIVERS_PRESSURE_SENSOR_DRIVER_H
#define USERDRIVERS_PRESSURE_SENSOR_DRIVER_H

#include <stdint.h>
#include "BSP/I2c/bsp_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USER_PRESSURE_SENSOR_ADDR_7BIT 0x78U
#define USER_PRESSURE_SENSOR_CMD_MEASURE 0xACU

typedef void (*UserPressureDelayMs)(uint32_t ms);

typedef struct
{
    uint8_t raw[6];
    uint8_t status;
    uint16_t pressure_raw;
    uint8_t temperature_raw;
} UserPressureSensorSample;

typedef struct
{
    BspI2c *left_bus;
    BspI2c *right_bus;
    UserPressureDelayMs delay_ms;
    uint8_t addr_7bit;
    uint8_t measure_cmd;
    uint8_t retry_count;
    uint32_t timeout_ms;
    UserPressureSensorSample left;
    UserPressureSensorSample right;
} UserPressureSensor;

void UserPressureSensor_Init(UserPressureSensor *dev,
                             BspI2c *left_bus,
                             BspI2c *right_bus,
                             UserPressureDelayMs delay_ms);
uint8_t UserPressureSensor_ReadOne(UserPressureSensor *dev,
                                   BspI2c *bus,
                                   UserPressureSensorSample *sample);
uint8_t UserPressureSensor_Read(UserPressureSensor *dev);

#ifdef __cplusplus
}
#endif

#endif
