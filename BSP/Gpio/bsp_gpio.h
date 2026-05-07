#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPIO 中断回调函数。
 *
 * 当某个 GPIO 外部中断触发时，用户可以通过这个回调拿到对应的
 * `BspGpioPin` 对象，从而在上层模块里处理按键、编码器、限位开关等事件。
 */
typedef void (*BspGpioExtiCallback)(void *context);

/**
 * @brief GPIO 引脚配置。
 *
 * 一个配置对象描述一个物理 GPIO 引脚。这样做的好处是上层模块不需要
 * 到处传 `GPIO_TypeDef *` 和 `pin`，而是把一个引脚当成一个可复用对象。
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state;
} BspGpioPinConfig;

/**
 * @brief GPIO 引脚对象。
 *
 * 每个 `BspGpioPin` 对应一个物理引脚。多个 LED、按键、片选脚都可以各自
 * 建一个对象，外部模块通过对象访问完整 GPIO 能力。
 */
typedef struct
{
    BspGpioPinConfig config;
    BspGpioExtiCallback exti_callback;
    void *exti_context;
} BspGpioPin;

void BspGpio_Init(BspGpioPin *pin, const BspGpioPinConfig *config);
void BspGpio_DeInit(BspGpioPin *pin);
void BspGpio_ConfigOutput(BspGpioPin *pin,
                          uint32_t mode,
                          uint32_t pull,
                          uint32_t speed);
void BspGpio_ConfigInput(BspGpioPin *pin, uint32_t pull);

void BspGpio_Write(BspGpioPin *pin, GPIO_PinState state);
void BspGpio_WriteActive(BspGpioPin *pin, uint8_t active);
void BspGpio_Set(BspGpioPin *pin);
void BspGpio_Reset(BspGpioPin *pin);
void BspGpio_Toggle(BspGpioPin *pin);

GPIO_PinState BspGpio_Read(BspGpioPin *pin);
uint8_t BspGpio_ReadActive(BspGpioPin *pin);

void BspGpio_SetExtiCallback(BspGpioPin *pin,
                             BspGpioExtiCallback callback,
                             void *context);
void BspGpio_OnExtiFromIsr(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif
