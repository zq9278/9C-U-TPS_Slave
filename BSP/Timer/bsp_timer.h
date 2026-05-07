#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 定时器周期中断回调。
 */
typedef void (*BspTimerPeriodElapsedCallback)(void *context);

/**
 * @brief 基础定时器配置。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;
} BspTimerConfig;

/**
 * @brief 基础定时器对象。
 *
 * 适用于周期调度、软定时节拍、运行时间计数等场景。PWM 请使用
 * `BspPwmChannel`，避免一个对象同时承担两类职责。
 */
typedef struct
{
    BspTimerConfig config;
    BspTimerPeriodElapsedCallback period_elapsed_callback;
    void *period_elapsed_context;
    uint8_t started;
} BspTimer;

void BspTimer_Init(BspTimer *timer, const BspTimerConfig *config);

uint8_t BspTimer_Start(BspTimer *timer);
uint8_t BspTimer_StartInterrupt(BspTimer *timer);
uint8_t BspTimer_Stop(BspTimer *timer);
uint8_t BspTimer_StopInterrupt(BspTimer *timer);

void BspTimer_SetCounter(BspTimer *timer, uint32_t counter);
uint32_t BspTimer_GetCounter(BspTimer *timer);
void BspTimer_ResetCounter(BspTimer *timer);

void BspTimer_SetPeriod(BspTimer *timer, uint32_t period_ticks);
uint32_t BspTimer_GetPeriod(BspTimer *timer);
void BspTimer_SetPrescaler(BspTimer *timer, uint32_t prescaler);
uint32_t BspTimer_GetPrescaler(BspTimer *timer);

void BspTimer_SetPeriodElapsedCallback(BspTimer *timer,
                                       BspTimerPeriodElapsedCallback callback,
                                       void *context);
void BspTimer_OnPeriodElapsedFromIsr(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif
