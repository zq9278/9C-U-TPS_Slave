#include "bsp_pwm.h"

#include <stddef.h>

/**
 * @brief 初始化 PWM 通道对象。
 *
 * 这个函数保存定时器句柄和通道号。定时器时钟、GPIO 复用、PWM 模式仍由
 * CubeMX 生成的 `MX_TIMx_Init()` 完成。
 */
void BspPwm_Init(BspPwmChannel *pwm, const BspPwmChannelConfig *config)
{
    if ((pwm == NULL) || (config == NULL))
    {
        return;
    }

    pwm->config = *config;
    pwm->started = 0U;
}

/**
 * @brief 启动 PWM 输出。
 *
 * 返回 1 表示 HAL 启动成功，返回 0 表示参数错误或 HAL 启动失败。
 */
uint8_t BspPwm_Start(BspPwmChannel *pwm)
{
    if ((pwm == NULL) || (pwm->config.htim == NULL))
    {
        return 0U;
    }

    if (pwm->started != 0U)
    {
        return 1U;
    }

    if (HAL_TIM_PWM_Start(pwm->config.htim, pwm->config.channel) != HAL_OK)
    {
        return 0U;
    }

    pwm->started = 1U;
    return 1U;
}

/**
 * @brief 停止 PWM 输出。
 */
uint8_t BspPwm_Stop(BspPwmChannel *pwm)
{
    if ((pwm == NULL) || (pwm->config.htim == NULL))
    {
        return 0U;
    }

    if (pwm->started == 0U)
    {
        return 1U;
    }

    if (HAL_TIM_PWM_Stop(pwm->config.htim, pwm->config.channel) != HAL_OK)
    {
        return 0U;
    }

    pwm->started = 0U;
    return 1U;
}

/**
 * @brief 直接设置比较值 CCR。
 */
void BspPwm_SetPulse(BspPwmChannel *pwm, uint32_t pulse)
{
    if ((pwm == NULL) || (pwm->config.htim == NULL))
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(pwm->config.htim, pwm->config.channel, pulse);
}

/**
 * @brief 读取当前比较值 CCR。
 */
uint32_t BspPwm_GetPulse(BspPwmChannel *pwm)
{
    if ((pwm == NULL) || (pwm->config.htim == NULL))
    {
        return 0U;
    }

    return __HAL_TIM_GET_COMPARE(pwm->config.htim, pwm->config.channel);
}

/**
 * @brief 设置 PWM 周期，单位是定时器 tick。
 *
 * 输入的是“周期 tick 数”，内部写入 ARR 时会减 1。调用者需要确保
 * 定时器当前预分频已经配置好。
 */
void BspPwm_SetPeriodTicks(BspPwmChannel *pwm, uint32_t period_ticks)
{
    if ((pwm == NULL) || (pwm->config.htim == NULL) || (period_ticks == 0U))
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(pwm->config.htim, period_ticks - 1U);
}

/**
 * @brief 获取 PWM 周期 tick 数。
 */
uint32_t BspPwm_GetPeriodTicks(BspPwmChannel *pwm)
{
    if ((pwm == NULL) || (pwm->config.htim == NULL))
    {
        return 0U;
    }

    return __HAL_TIM_GET_AUTORELOAD(pwm->config.htim) + 1U;
}

/**
 * @brief 设置 PWM 占空比，单位为千分比。
 *
 * `0` 表示 0%，`1000` 表示 100%。用整数千分比避免在 BSP 层引入浮点依赖。
 */
void BspPwm_SetDutyPermille(BspPwmChannel *pwm, uint16_t duty_permille)
{
    uint32_t period_ticks;
    uint32_t pulse;

    if (duty_permille > 1000U)
    {
        duty_permille = 1000U;
    }

    period_ticks = BspPwm_GetPeriodTicks(pwm);
    if (period_ticks == 0U)
    {
        return;
    }

    pulse = (period_ticks * (uint32_t)duty_permille) / 1000U;
    BspPwm_SetPulse(pwm, pulse);
}

/**
 * @brief 获取 PWM 占空比，单位为千分比。
 */
uint16_t BspPwm_GetDutyPermille(BspPwmChannel *pwm)
{
    uint32_t period_ticks = BspPwm_GetPeriodTicks(pwm);
    uint32_t pulse = BspPwm_GetPulse(pwm);

    if (period_ticks == 0U)
    {
        return 0U;
    }

    return (uint16_t)((pulse * 1000U) / period_ticks);
}
