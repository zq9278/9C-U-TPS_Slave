#include "heat.h"
#include "tim.h"
#include "pid.h"


//u16 HeatPWMVal=0;//0-1999

extern TIM_HandleTypeDef htim14;

void OTP_Reset(u8 channel)
{
	if(channel==1)
	{
		OTP1_RESET_OUT();
		OTP1_RESET(1);//复位过热保护
		osDelay(50);
		OTP1_RESET_IN();
	}
	else if(channel==2)
	{
		OTP2_RESET_OUT();
		OTP2_RESET(1);//复位过热保护
		osDelay(50);
		OTP2_RESET_IN();
	}
}

void HeatPower(u8 channel,u8 state)
{
	if(channel==1)
	{
		if(state==1)
		{
			OTP_Reset(channel);
			HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
			OTP1_RESET_OUT();
			HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port,OTP1_RESET_Pin,GPIO_PIN_RESET);//关闭加热mos管
		}
	}
	else if(channel==2)
	{
		if(state==1)
		{
			OTP_Reset(channel);
			HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
			OTP2_RESET_OUT();
			HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port,OTP2_RESET_Pin,GPIO_PIN_RESET);//关闭加热mos管
		}
	}
}

void HeatPWMSet(u8 channel,u16 PWMVal)
{
	if(channel==1)
	{
		TIM14->CCR1=PWMVal;
	}
	else if(channel==2)
	{
		TIM17->CCR1=PWMVal;
	}
}

int32_t tempb;
void PID_Heat(u8 channel,PID_TypeDef *pid,int32_t measured_value)
{
	tempb=PID_Compute(pid,measured_value);
	HeatPWMSet(channel,(u16)tempb);
}





















