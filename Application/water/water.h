#ifndef _WATER_H
#define _WATER_H
#include <sys.h>	  

#define WaterPump1(n)  (n?HAL_GPIO_WritePin(WaterPump1_GPIO_Port,WaterPump1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WaterPump1_GPIO_Port,WaterPump1_Pin,GPIO_PIN_RESET)) 
#define AirPump3(n)  (n?HAL_GPIO_WritePin(AirPump3_GPIO_Port,AirPump3_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(AirPump3_GPIO_Port,AirPump3_Pin,GPIO_PIN_RESET)) 
#define Fan1(n)  (n?HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_RESET)) 
#define WaterValve1(n)  (n?HAL_GPIO_WritePin(WaterValve1_GPIO_Port,WaterValve1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WaterValve1_GPIO_Port,WaterValve1_Pin,GPIO_PIN_RESET)) 
#define WaterValve2(n)  (n?HAL_GPIO_WritePin(WaterValve2_GPIO_Port,WaterValve2_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WaterValve2_GPIO_Port,WaterValve2_Pin,GPIO_PIN_RESET)) 
#define WaterValve3(n)  (n?HAL_GPIO_WritePin(WaterValve3_GPIO_Port,WaterValve3_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WaterValve3_GPIO_Port,WaterValve3_Pin,GPIO_PIN_RESET)) 
#define WaterValve4(n)  (n?HAL_GPIO_WritePin(WaterValve4_GPIO_Port,WaterValve4_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WaterValve4_GPIO_Port,WaterValve4_Pin,GPIO_PIN_RESET)) 
#define WaterValve5(n)  (n?HAL_GPIO_WritePin(WaterValve5_GPIO_Port,WaterValve5_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(WaterValve5_GPIO_Port,WaterValve5_Pin,GPIO_PIN_RESET)) 


u8 StartWaterPump(void);
void StopWaterPump(void);
void StartIPLCold(u8 ColdValue);
void StopIPLCold(void);

#endif

