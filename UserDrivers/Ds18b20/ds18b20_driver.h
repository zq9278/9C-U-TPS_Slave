#ifndef USERDRIVERS_DS18B20_DRIVER_H
#define USERDRIVERS_DS18B20_DRIVER_H

#include <stdint.h>
#include "BSP/Gpio/bsp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UserDs18b20DelayUs)(uint16_t us);
typedef void (*UserDs18b20CriticalHook)(void);

typedef struct
{
    BspGpioPin *dq;
    UserDs18b20DelayUs delay_us;
    UserDs18b20CriticalHook enter_critical;
    UserDs18b20CriticalHook exit_critical;
} UserDs18b20;

void UserDs18b20_InitDevice(UserDs18b20 *dev,
                            BspGpioPin *dq,
                            UserDs18b20DelayUs delay_us,
                            UserDs18b20CriticalHook enter_critical,
                            UserDs18b20CriticalHook exit_critical);
uint8_t UserDs18b20_Init(UserDs18b20 *dev);
void UserDs18b20_Reset(UserDs18b20 *dev);
uint8_t UserDs18b20_Check(UserDs18b20 *dev);
void UserDs18b20_Start(UserDs18b20 *dev);
void UserDs18b20_WriteByte(UserDs18b20 *dev, uint8_t value);
uint8_t UserDs18b20_ReadBit(UserDs18b20 *dev);
uint8_t UserDs18b20_ReadByte(UserDs18b20 *dev);
int16_t UserDs18b20_GetTempX10(UserDs18b20 *dev);

#ifdef __cplusplus
}
#endif

#endif
