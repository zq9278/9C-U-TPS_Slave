#include "ds18b20_driver.h"

#include <stddef.h>

static void DsDelay(UserDs18b20 *dev, uint16_t us)
{
    if ((dev != NULL) && (dev->delay_us != NULL))
    {
        dev->delay_us(us);
    }
}

static void DsEnter(UserDs18b20 *dev)
{
    if ((dev != NULL) && (dev->enter_critical != NULL))
    {
        dev->enter_critical();
    }
}

static void DsExit(UserDs18b20 *dev)
{
    if ((dev != NULL) && (dev->exit_critical != NULL))
    {
        dev->exit_critical();
    }
}

static void DsLineOutput(UserDs18b20 *dev)
{
    BspGpio_ConfigOutput(dev->dq, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH);
}

static void DsLineInput(UserDs18b20 *dev)
{
    BspGpio_ConfigInput(dev->dq, GPIO_PULLUP);
}

void UserDs18b20_InitDevice(UserDs18b20 *dev,
                            BspGpioPin *dq,
                            UserDs18b20DelayUs delay_us,
                            UserDs18b20CriticalHook enter_critical,
                            UserDs18b20CriticalHook exit_critical)
{
    if (dev == NULL)
    {
        return;
    }

    dev->dq = dq;
    dev->delay_us = delay_us;
    dev->enter_critical = enter_critical;
    dev->exit_critical = exit_critical;
}

void UserDs18b20_Reset(UserDs18b20 *dev)
{
    if ((dev == NULL) || (dev->dq == NULL))
    {
        return;
    }

    DsEnter(dev);
    DsLineOutput(dev);
    BspGpio_Reset(dev->dq);
    DsDelay(dev, 750U);
    BspGpio_Set(dev->dq);
    DsDelay(dev, 15U);
    DsExit(dev);
}

uint8_t UserDs18b20_Check(UserDs18b20 *dev)
{
    uint8_t retry = 0U;

    if ((dev == NULL) || (dev->dq == NULL))
    {
        return 1U;
    }

    DsLineInput(dev);
    while ((BspGpio_Read(dev->dq) != GPIO_PIN_RESET) && (retry < 200U))
    {
        ++retry;
        DsDelay(dev, 1U);
    }
    if (retry >= 200U)
    {
        return 1U;
    }

    retry = 0U;
    while ((BspGpio_Read(dev->dq) == GPIO_PIN_RESET) && (retry < 240U))
    {
        ++retry;
        DsDelay(dev, 1U);
    }

    return (uint8_t)(retry >= 240U);
}

uint8_t UserDs18b20_ReadBit(UserDs18b20 *dev)
{
    uint8_t value;

    if ((dev == NULL) || (dev->dq == NULL))
    {
        return 0U;
    }

    DsEnter(dev);
    DsLineOutput(dev);
    BspGpio_Reset(dev->dq);
    DsDelay(dev, 2U);
    BspGpio_Set(dev->dq);
    DsLineInput(dev);
    DsDelay(dev, 12U);
    value = (uint8_t)(BspGpio_Read(dev->dq) == GPIO_PIN_SET);
    DsExit(dev);

    DsDelay(dev, 50U);
    return value;
}

uint8_t UserDs18b20_ReadByte(UserDs18b20 *dev)
{
    uint8_t i;
    uint8_t value = 0U;

    for (i = 0U; i < 8U; ++i)
    {
        value = (uint8_t)((UserDs18b20_ReadBit(dev) << 7U) | (value >> 1U));
    }

    return value;
}

void UserDs18b20_WriteByte(UserDs18b20 *dev, uint8_t value)
{
    uint8_t i;

    if ((dev == NULL) || (dev->dq == NULL))
    {
        return;
    }

    DsEnter(dev);
    DsLineOutput(dev);
    for (i = 0U; i < 8U; ++i)
    {
        if ((value & 0x01U) != 0U)
        {
            BspGpio_Reset(dev->dq);
            DsDelay(dev, 2U);
            BspGpio_Set(dev->dq);
            DsDelay(dev, 60U);
        }
        else
        {
            BspGpio_Reset(dev->dq);
            DsDelay(dev, 60U);
            BspGpio_Set(dev->dq);
            DsDelay(dev, 2U);
        }
        value >>= 1U;
    }
    DsExit(dev);
}

void UserDs18b20_Start(UserDs18b20 *dev)
{
    UserDs18b20_Reset(dev);
    (void)UserDs18b20_Check(dev);
    UserDs18b20_WriteByte(dev, 0xCCU);
    UserDs18b20_WriteByte(dev, 0x44U);
}

uint8_t UserDs18b20_Init(UserDs18b20 *dev)
{
    if ((dev == NULL) || (dev->dq == NULL))
    {
        return 1U;
    }

    DsLineOutput(dev);
    BspGpio_Set(dev->dq);
    UserDs18b20_Reset(dev);
    return UserDs18b20_Check(dev);
}

int16_t UserDs18b20_GetTempX10(UserDs18b20 *dev)
{
    uint8_t tl;
    uint8_t th;
    int16_t raw;
    int32_t temp_x10;

    UserDs18b20_Start(dev);
    UserDs18b20_Reset(dev);
    (void)UserDs18b20_Check(dev);
    UserDs18b20_WriteByte(dev, 0xCCU);
    UserDs18b20_WriteByte(dev, 0xBEU);

    tl = UserDs18b20_ReadByte(dev);
    th = UserDs18b20_ReadByte(dev);
    raw = (int16_t)(((uint16_t)th << 8U) | tl);
    temp_x10 = ((int32_t)raw * 10L) / 16L;

    return (int16_t)temp_x10;
}
