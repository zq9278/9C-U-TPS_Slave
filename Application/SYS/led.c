#include "led.h"
#include "tim.h"

u8 LEDNub,LastLEDNub;
u16 LEDPWMVal=0;
//u8 LEDDir;
u8 LEDState,LastLEDState;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern u8 BQ25895Reg[21];

extern u8 PowerState,LastPowerState;

void LEDPWMSet(u8 Nub,u16 PWMVal)
{
	switch(Nub)
	{
		case LED2:
			TIM16->CCR1=PWMVal;
			break;
		case LED3:
			TIM17->CCR1=PWMVal;
			break;
		case LED4:
			TIM3->CCR3=PWMVal;
			break;
		default:break;
	}
}

void LEDCtrl(u8 Nub,u8 State)
{
	if(State==LEDPWM || State==LEDOn)
	{
		switch(Nub)
		{
			case LED2:
				HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
				break;
			case LED3:
				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
				break;
			case LED4:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
				break;
			default:break;
		}
	}
	else if(State==LEDOff)
	{
		HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
	}
	if(State==LEDOn)
	{
		LEDPWMSet(Nub,500);
	}
	LEDNub=Nub;
	LEDState=State;
	LastLEDNub=LEDNub;
	LastLEDState=LEDState;
}

//void LEDUpdate(void)
//{
//	if(LastPowerState!=PowerState)
//	{
//		switch(PowerState)
//		{
//			case 1:
//				LEDCtrl(LED2,LEDPWM);
//				break;
//			case 2:
//				LEDCtrl(LED2,LEDOn);
//				break;
//			case 3:
//				LEDCtrl(LED3,LEDOn);
//				break;
//			default:break;
//		}
//	}
//	LastPowerState=PowerState;
//}




























