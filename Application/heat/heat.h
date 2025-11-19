#ifndef _HEAT_H
#define _HEAT_H
#include <sys.h>	  
#include "pid.h"

//IO方向设置
#define OTP1_RESET_IN()  {OTP1_RESET_GPIO_Port->MODER&=~(GPIO_MODER_MODE0<<(6*2));OTP1_RESET_GPIO_Port->MODER|=0<<6*2;}	//PA6输入模式
#define OTP1_RESET_OUT() {OTP1_RESET_GPIO_Port->MODER&=~(GPIO_MODER_MODE0<<(6*2));OTP1_RESET_GPIO_Port->MODER|=1<<6*2;} //PA6输出模式
#define OTP2_RESET_IN()  {OTP2_RESET_GPIO_Port->MODER&=~(GPIO_MODER_MODE0<<(1*2));OTP2_RESET_GPIO_Port->MODER|=0<<1*2;}	//PB1输入模式
#define OTP2_RESET_OUT() {OTP2_RESET_GPIO_Port->MODER&=~(GPIO_MODER_MODE0<<(1*2));OTP2_RESET_GPIO_Port->MODER|=1<<1*2;} //PB1输出模式

//IO操作
#define OTP1_RESET(n)  (n?HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port,OTP1_RESET_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port,OTP1_RESET_Pin,GPIO_PIN_RESET)) //OTP1_RESET
#define OTP2_RESET(n)  (n?HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port,OTP2_RESET_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port,OTP2_RESET_Pin,GPIO_PIN_RESET)) //OTP2_RESET

#define Right 1
#define Left 2

void OTP_Reset(u8 channel);
void HeatPower(u8 channel,u8 state);
void HeatPWMSet(u8 channel,u16 PWMVal);
void PID_Heat(u8 channel,PID_TypeDef *pid,int32_t measured_value);

#endif

