#include "pressure_sensor_driver.h"

#include <stddef.h>
#include <string.h>

static void PressureDelay(UserPressureSensor *dev, uint32_t ms)
{
    if ((dev != NULL) && (dev->delay_ms != NULL))
    {
        dev->delay_ms(ms);
    }
}

void UserPressureSensor_Init(UserPressureSensor *dev,
                             BspI2c *left_bus,
                             BspI2c *right_bus,
                             UserPressureDelayMs delay_ms)
{
    if (dev == NULL)
    {
        return;
    }

    (void)memset(dev, 0, sizeof(*dev));
    dev->left_bus = left_bus;
    dev->right_bus = right_bus;
    dev->delay_ms = delay_ms;
    dev->addr_7bit = USER_PRESSURE_SENSOR_ADDR_7BIT;
    dev->measure_cmd = USER_PRESSURE_SENSOR_CMD_MEASURE;
    dev->retry_count = 2U;
    dev->timeout_ms = 20U;
}

uint8_t UserPressureSensor_ReadOne(UserPressureSensor *dev,
                                   BspI2c *bus,
                                   UserPressureSensorSample *sample)
{
    uint8_t attempt;

    if ((dev == NULL) || (bus == NULL) || (sample == NULL))
    {
        return 0U;
    }

    for (attempt = 0U; attempt < dev->retry_count; ++attempt)
    {
        uint32_t pressure_raw;

        if (BspI2c_MasterReceive(bus, dev->addr_7bit, &sample->status, 1U, dev->timeout_ms) == 0U)
        {
            BspI2c_Recover(bus);
            continue;
        }

        if ((sample->status & (1U << 5U)) == 0U)
        {
            if (BspI2c_MasterTransmit(bus, dev->addr_7bit, &dev->measure_cmd, 1U, dev->timeout_ms) == 0U)
            {
                BspI2c_Recover(bus);
                continue;
            }
            PressureDelay(dev, 10U);
        }

        if (BspI2c_MasterReceive(bus, dev->addr_7bit, sample->raw, 6U, dev->timeout_ms) == 0U)
        {
            BspI2c_Recover(bus);
            continue;
        }

        pressure_raw = ((uint32_t)sample->raw[1] << 16U) |
                       ((uint32_t)sample->raw[2] << 8U) |
                       (uint32_t)sample->raw[3];
        if (pressure_raw == 0xFFFFFFU)
        {
            BspI2c_Recover(bus);
            continue;
        }

        sample->pressure_raw = (uint16_t)(((pressure_raw >> 8U) * 100000U) / 65536U);
        sample->temperature_raw = sample->raw[4];
        return 1U;
    }

    return 0U;
}

uint8_t UserPressureSensor_Read(UserPressureSensor *dev)
{
    UserPressureSensorSample last_left;
    UserPressureSensorSample last_right;
    uint8_t left_ok;
    uint8_t right_ok;

    if (dev == NULL)
    {
        return 0U;
    }

    last_left = dev->left;
    last_right = dev->right;

    left_ok = UserPressureSensor_ReadOne(dev, dev->left_bus, &dev->left);
    if (left_ok == 0U)
    {
        dev->left = last_left;
    }

    if (dev->right_bus != NULL)
    {
        right_ok = UserPressureSensor_ReadOne(dev, dev->right_bus, &dev->right);
        if (right_ok == 0U)
        {
            dev->right = last_right;
        }
    }
    else
    {
        dev->right = dev->left;
        right_ok = left_ok;
    }

    return (uint8_t)(left_ok && right_ok);
}
