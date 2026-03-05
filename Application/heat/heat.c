#include "heat.h"
#include "tim.h"
#include "pid.h"

/*
 * heat.c
 * - HeatPower: 仅控制加热通道启停，不触发 OTP_Reset
 * - HeatPWMSet: 设置加热 PWM
 * - PID_Heat: 温控 PID 输出到 PWM
 */

extern TIM_HandleTypeDef htim14;

/* 记录通道当前是否处于 PWM 运行状态，避免重复 Start */
static uint8_t s_heat_on_ch1 = 0;
static uint8_t s_heat_on_ch2 = 0;

void OTP_Reset(u8 channel)
{
    if (channel == 1)
    {
        OTP1_RESET_OUT();
        OTP1_RESET(1);
        osDelay(50);
        OTP1_RESET_IN();
    }
    else if (channel == 2)
    {
        OTP2_RESET_OUT();
        OTP2_RESET(1);
        osDelay(50);
        OTP2_RESET_IN();
    }
}

/*
 * 仅负责通道启停：
 * state=1 启动 PWM；state=0 停止 PWM 并关闭 MOS
 */
void HeatPower(u8 channel, u8 state)
{
    if (channel == 1)
    {
        if (state == 1)
        {
            if (!s_heat_on_ch1) {
                HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
                s_heat_on_ch1 = 1;
            }
        }
        else
        {
            HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
            OTP1_RESET_OUT();
            HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_RESET);
            s_heat_on_ch1 = 0;
        }
    }
    else if (channel == 2)
    {
        if (state == 1)
        {
            if (!s_heat_on_ch2) {
                HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
                s_heat_on_ch2 = 1;
            }
        }
        else
        {
            HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
            OTP2_RESET_OUT();
            HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_RESET);
            s_heat_on_ch2 = 0;
        }
    }
}

void HeatPWMSet(u8 channel, u16 PWMVal)
{
    if (channel == 1)
    {
        TIM14->CCR1 = PWMVal;
    }
    else if (channel == 2)
    {
        TIM17->CCR1 = PWMVal;
    }
}

int32_t tempb;

void PID_Heat(u8 channel, PID_TypeDef *pid, int32_t measured_value)
{
    tempb = PID_Compute(pid, measured_value);
    HeatPWMSet(channel, (u16)tempb);
}