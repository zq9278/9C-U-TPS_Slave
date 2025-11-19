#ifndef _ADS1248_H
#define _ADS1248_H
#include <sys.h>	

#define RTD_CS(n)  (n?HAL_GPIO_WritePin(RTD_CS_GPIO_Port,RTD_CS_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RTD_CS_GPIO_Port,RTD_CS_Pin,GPIO_PIN_RESET)) 
#define RTD_START(n)  (n?HAL_GPIO_WritePin(RTD_START_GPIO_Port,RTD_START_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RTD_START_GPIO_Port,RTD_START_Pin,GPIO_PIN_RESET)) 

#define RTD1 0
#define RTD2 1

#define R_REF 30000       // 参考电阻，单位：欧姆
#define ADC_MAX 8388608   // 2^23

#define TEMP_TABLE_MID_SIZE 154
#define TEMP_TABLE_HOT_SIZE 25
#define TEMP_TABLE_COLD_SIZE 25

#define ADC_CMD_RDATA 0x12
#define ADC_CMD_RREG 0x20
#define ADC_CMD_WREG 0x40
#define ADC_CMD_NOP 0xff

void ADS1248_Init(void);
void ADS1248_ChangeChannel(u8 RTCNum);
u32 ADS1248_Read(void);
u16 ADC2Temperature(u32 adc_code);

#endif
