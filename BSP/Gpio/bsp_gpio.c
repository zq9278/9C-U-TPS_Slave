#include "bsp_gpio.h"

#include <stddef.h>

enum
{
    BSP_GPIO_EXTI_SLOT_COUNT = 16U
};

static BspGpioPin *g_exti_slots[BSP_GPIO_EXTI_SLOT_COUNT];

/**
 * @brief 根据 GPIO_Pin 掩码找到 EXTI 注册槽位。
 *
 * STM32 的 GPIO pin 通常是 `GPIO_PIN_0` 到 `GPIO_PIN_15` 这种单 bit 掩码。
 * 这里把掩码转成 0~15 的索引，便于在中断回调里快速找到对应对象。
 */
static int32_t BspGpio_PinToIndex(uint16_t gpio_pin)
{
    uint32_t index;

    for (index = 0U; index < BSP_GPIO_EXTI_SLOT_COUNT; ++index)
    {
        if (gpio_pin == (uint16_t)(1UL << index))
        {
            return (int32_t)index;
        }
    }

    return -1;
}

/**
 * @brief 初始化一个 GPIO 对象。
 *
 * 这个函数只保存对象配置，不重新配置 GPIO 模式。GPIO 的模式、上下拉、
 * 速度和复用功能仍由 CubeMX 在 `MX_GPIO_Init()` 中统一生成。
 */
void BspGpio_Init(BspGpioPin *pin, const BspGpioPinConfig *config)
{
    if ((pin == NULL) || (config == NULL))
    {
        return;
    }

    pin->config = *config;
    pin->exti_callback = NULL;
    pin->exti_context = NULL;
}

/**
 * @brief 反初始化一个 GPIO 对象对应的硬件引脚。
 *
 * 注意：这个函数会调用 `HAL_GPIO_DeInit()`，会影响该物理引脚的硬件配置。
 * 如果这个引脚由 CubeMX 或其他模块继续使用，就不要随意调用。
 */
void BspGpio_DeInit(BspGpioPin *pin)
{
    if ((pin == NULL) || (pin->config.port == NULL))
    {
        return;
    }

    HAL_GPIO_DeInit(pin->config.port, pin->config.pin);
}

void BspGpio_ConfigOutput(BspGpioPin *pin,
                          uint32_t mode,
                          uint32_t pull,
                          uint32_t speed)
{
    GPIO_InitTypeDef gpio = {0};

    if ((pin == NULL) || (pin->config.port == NULL))
    {
        return;
    }

    gpio.Pin = pin->config.pin;
    gpio.Mode = mode;
    gpio.Pull = pull;
    gpio.Speed = speed;
    HAL_GPIO_Init(pin->config.port, &gpio);
}

void BspGpio_ConfigInput(BspGpioPin *pin, uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};

    if ((pin == NULL) || (pin->config.port == NULL))
    {
        return;
    }

    gpio.Pin = pin->config.pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(pin->config.port, &gpio);
}

/**
 * @brief 按原始电平写 GPIO。
 */
void BspGpio_Write(BspGpioPin *pin, GPIO_PinState state)
{
    if ((pin == NULL) || (pin->config.port == NULL))
    {
        return;
    }

    HAL_GPIO_WritePin(pin->config.port, pin->config.pin, state);
}

/**
 * @brief 按业务有效状态写 GPIO。
 *
 * 例如低电平点亮的 LED 可以把 `active_state` 配成 `GPIO_PIN_RESET`，
 * 上层仍然调用 `BspGpio_WriteActive(pin, 1)` 表示打开。
 */
void BspGpio_WriteActive(BspGpioPin *pin, uint8_t active)
{
    GPIO_PinState state;

    if (pin == NULL)
    {
        return;
    }

    state = (active != 0U) ? pin->config.active_state :
                             (GPIO_PinState)(pin->config.active_state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    BspGpio_Write(pin, state);
}

/**
 * @brief 输出高电平。
 */
void BspGpio_Set(BspGpioPin *pin)
{
    BspGpio_Write(pin, GPIO_PIN_SET);
}

/**
 * @brief 输出低电平。
 */
void BspGpio_Reset(BspGpioPin *pin)
{
    BspGpio_Write(pin, GPIO_PIN_RESET);
}

/**
 * @brief 翻转 GPIO 输出电平。
 */
void BspGpio_Toggle(BspGpioPin *pin)
{
    if ((pin == NULL) || (pin->config.port == NULL))
    {
        return;
    }

    HAL_GPIO_TogglePin(pin->config.port, pin->config.pin);
}

/**
 * @brief 读取 GPIO 原始电平。
 */
GPIO_PinState BspGpio_Read(BspGpioPin *pin)
{
    if ((pin == NULL) || (pin->config.port == NULL))
    {
        return GPIO_PIN_RESET;
    }

    return HAL_GPIO_ReadPin(pin->config.port, pin->config.pin);
}

/**
 * @brief 读取 GPIO 是否处于业务有效状态。
 */
uint8_t BspGpio_ReadActive(BspGpioPin *pin)
{
    if (pin == NULL)
    {
        return 0U;
    }

    return (uint8_t)(BspGpio_Read(pin) == pin->config.active_state);
}

/**
 * @brief 注册 GPIO 外部中断回调。
 *
 * 用户仍需要在 `HAL_GPIO_EXTI_Callback()` 中调用 `BspGpio_OnExtiFromIsr()`，
 * BSP 层才能把 HAL 的 pin 中断转发到具体的 `BspGpioPin` 对象。
 */
void BspGpio_SetExtiCallback(BspGpioPin *pin,
                             BspGpioExtiCallback callback,
                             void *context)
{
    int32_t index;

    if (pin == NULL)
    {
        return;
    }

    pin->exti_callback = callback;
    pin->exti_context = context;

    index = BspGpio_PinToIndex(pin->config.pin);
    if (index >= 0)
    {
        g_exti_slots[index] = pin;
    }
}

/**
 * @brief 从 HAL GPIO EXTI 中断回调中转发事件。
 *
 * 推荐在工程的 `HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)` 中调用本函数。
 */
void BspGpio_OnExtiFromIsr(uint16_t gpio_pin)
{
    int32_t index = BspGpio_PinToIndex(gpio_pin);
    BspGpioPin *pin;

    if (index < 0)
    {
        return;
    }

    pin = g_exti_slots[index];
    if ((pin != NULL) && (pin->exti_callback != NULL))
    {
        pin->exti_callback(pin->exti_context);
    }
}
