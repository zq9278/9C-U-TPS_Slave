#include "water.h"

extern TIM_HandleTypeDef htim15;

volatile u8 WaterState=0;
/*
0000 0xyz
x:0,未在加水
  1,正在加水
y:0，未在排水
  1,正在排水
z:0，未在循环
  1，正在循环
*/

u8 StartWaterPump(void)
{
	if(HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)==0)
	{
		WaterPump1(1);
		WaterState=WaterState|0x01;
		return 1;
	}
	else
	{
		return 0;
	}
}

void StopWaterPump(void)
{
	WaterPump1(0);
	WaterState=WaterState&0xfe;
}

void StartIPLCold(u8 ColdValue)
{
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
	if(ColdValue>190)ColdValue=190;
	TIM15->CCR2=ColdValue;
}

void StopIPLCold(void)
{
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_2);
}
