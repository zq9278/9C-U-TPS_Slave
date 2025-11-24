#ifndef _ADS1248_H
#define _ADS1248_H
#include <sys.h>	
#include <sys/_intsup.h>

#define RTD_CS(n)  (n?HAL_GPIO_WritePin(RTD_CS_GPIO_Port,RTD_CS_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RTD_CS_GPIO_Port,RTD_CS_Pin,GPIO_PIN_RESET)) 
#define RTD_START(n)  (n?HAL_GPIO_WritePin(RTD_START_GPIO_Port,RTD_START_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(RTD_START_GPIO_Port,RTD_START_Pin,GPIO_PIN_RESET)) 

#define RTD1 0
#define RTD2 1

#define R_REF 30000       // �ο����裬��λ��ŷķ
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
int32_t ADS1248_Read(void);
u16 ADC2Temperature(u32 adc_code);

// 简单多字节写寄存器
static void ADS1248_WriteRegs(uint8_t startReg, const uint8_t *data, uint8_t len);

// 可选：读寄存器（调试用）
static void ADS1248_ReadRegs(uint8_t startReg, uint8_t *data, uint8_t len);
// 自校准（命令码以你项目为准，这里沿用你的风格）
static void ADS1248_SelfCal(void);

#endif
