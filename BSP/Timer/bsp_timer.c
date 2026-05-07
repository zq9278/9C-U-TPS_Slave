#include "bsp_timer.h"

#include <stddef.h>

enum
{
    BSP_TIMER_CALLBACK_SLOT_COUNT = 16U
};

static BspTimer *g_timer_slots[BSP_TIMER_CALLBACK_SLOT_COUNT];

/**
 * @brief 在定时器回调注册表中查找对象。
 */
static int32_t BspTimer_FindSlot(TIM_HandleTypeDef *htim)
{
    uint32_t index;

    for (index = 0U; index < BSP_TIMER_CALLBACK_SLOT_COUNT; ++index)
    {
        if ((g_timer_slots[index] != NULL) &&
            (g_timer_slots[index]->config.htim == htim))
        {
            return (int32_t)index;
        }
    }

    return -1;
}

/**
 * @brief 为一个定时器对象分配或更新回调注册槽位。
 */
static void BspTimer_RegisterSlot(BspTimer *timer)
{
    uint32_t index;
    int32_t old_slot;

    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return;
    }

    old_slot = BspTimer_FindSlot(timer->config.htim);
    if (old_slot >= 0)
    {
        g_timer_slots[old_slot] = timer;
        return;
    }

    for (index = 0U; index < BSP_TIMER_CALLBACK_SLOT_COUNT; ++index)
    {
        if (g_timer_slots[index] == NULL)
        {
            g_timer_slots[index] = timer;
            return;
        }
    }
}

/**
 * @brief 初始化基础定时器对象。
 */
void BspTimer_Init(BspTimer *timer, const BspTimerConfig *config)
{
    if ((timer == NULL) || (config == NULL))
    {
        return;
    }

    timer->config = *config;
    timer->period_elapsed_callback = NULL;
    timer->period_elapsed_context = NULL;
    timer->started = 0U;
}

/**
 * @brief 启动基础定时器，不开启更新中断。
 */
uint8_t BspTimer_Start(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    if (HAL_TIM_Base_Start(timer->config.htim) != HAL_OK)
    {
        return 0U;
    }

    timer->started = 1U;
    return 1U;
}

/**
 * @brief 启动基础定时器并开启周期中断。
 */
uint8_t BspTimer_StartInterrupt(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    BspTimer_RegisterSlot(timer);

    if (HAL_TIM_Base_Start_IT(timer->config.htim) != HAL_OK)
    {
        return 0U;
    }

    timer->started = 1U;
    return 1U;
}

/**
 * @brief 停止基础定时器。
 */
uint8_t BspTimer_Stop(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    if (HAL_TIM_Base_Stop(timer->config.htim) != HAL_OK)
    {
        return 0U;
    }

    timer->started = 0U;
    return 1U;
}

/**
 * @brief 停止基础定时器周期中断。
 */
uint8_t BspTimer_StopInterrupt(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    if (HAL_TIM_Base_Stop_IT(timer->config.htim) != HAL_OK)
    {
        return 0U;
    }

    timer->started = 0U;
    return 1U;
}

/**
 * @brief 设置当前计数值 CNT。
 */
void BspTimer_SetCounter(BspTimer *timer, uint32_t counter)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(timer->config.htim, counter);
}

/**
 * @brief 获取当前计数值 CNT。
 */
uint32_t BspTimer_GetCounter(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    return __HAL_TIM_GET_COUNTER(timer->config.htim);
}

/**
 * @brief 清零当前计数值。
 */
void BspTimer_ResetCounter(BspTimer *timer)
{
    BspTimer_SetCounter(timer, 0U);
}

/**
 * @brief 设置定时器周期，单位为 timer tick。
 */
void BspTimer_SetPeriod(BspTimer *timer, uint32_t period_ticks)
{
    if ((timer == NULL) || (timer->config.htim == NULL) || (period_ticks == 0U))
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(timer->config.htim, period_ticks - 1U);
}

/**
 * @brief 获取定时器周期 tick 数。
 */
uint32_t BspTimer_GetPeriod(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    return __HAL_TIM_GET_AUTORELOAD(timer->config.htim) + 1U;
}

/**
 * @brief 设置定时器预分频值。
 *
 * 这里传入的是实际写入 PSC 的值。例如要做 84 分频，通常写入 83。
 */
void BspTimer_SetPrescaler(BspTimer *timer, uint32_t prescaler)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return;
    }

    __HAL_TIM_SET_PRESCALER(timer->config.htim, prescaler);
}

/**
 * @brief 获取定时器预分频寄存器值。
 */
uint32_t BspTimer_GetPrescaler(BspTimer *timer)
{
    if ((timer == NULL) || (timer->config.htim == NULL))
    {
        return 0U;
    }

    return timer->config.htim->Instance->PSC;
}

/**
 * @brief 注册定时器周期中断回调。
 *
 * 用户需要在 `HAL_TIM_PeriodElapsedCallback()` 中调用
 * `BspTimer_OnPeriodElapsedFromIsr(htim)`，BSP 才能把 HAL 回调分发到对象。
 */
void BspTimer_SetPeriodElapsedCallback(BspTimer *timer,
                                       BspTimerPeriodElapsedCallback callback,
                                       void *context)
{
    if (timer == NULL)
    {
        return;
    }

    timer->period_elapsed_callback = callback;
    timer->period_elapsed_context = context;
    BspTimer_RegisterSlot(timer);
}

/**
 * @brief 从 HAL 定时器周期回调中转发事件。
 */
void BspTimer_OnPeriodElapsedFromIsr(TIM_HandleTypeDef *htim)
{
    int32_t slot = BspTimer_FindSlot(htim);
    BspTimer *timer;

    if (slot < 0)
    {
        return;
    }

    timer = g_timer_slots[slot];
    if ((timer != NULL) && (timer->period_elapsed_callback != NULL))
    {
        timer->period_elapsed_callback(timer->period_elapsed_context);
    }
}
