#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PWM 通道配置。
 *
 * 一个 `BspPwmChannel` 对象对应一个定时器的一个 PWM 通道，例如
 * `TIM3 + TIM_CHANNEL_1`。这样同一个驱动可以复用到蜂鸣器、风扇、背光、
 * 加热 PWM 等不同输出。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} BspPwmChannelConfig;

/**
 * @brief PWM 通道对象。
 */
typedef struct
{
    BspPwmChannelConfig config;
    uint8_t started;
} BspPwmChannel;

void BspPwm_Init(BspPwmChannel *pwm, const BspPwmChannelConfig *config);
uint8_t BspPwm_Start(BspPwmChannel *pwm);
uint8_t BspPwm_Stop(BspPwmChannel *pwm);

void BspPwm_SetPulse(BspPwmChannel *pwm, uint32_t pulse);
uint32_t BspPwm_GetPulse(BspPwmChannel *pwm);

void BspPwm_SetPeriodTicks(BspPwmChannel *pwm, uint32_t period_ticks);
uint32_t BspPwm_GetPeriodTicks(BspPwmChannel *pwm);

void BspPwm_SetDutyPermille(BspPwmChannel *pwm, uint16_t duty_permille);
uint16_t BspPwm_GetDutyPermille(BspPwmChannel *pwm);

#ifdef __cplusplus
}
#endif

#endif
