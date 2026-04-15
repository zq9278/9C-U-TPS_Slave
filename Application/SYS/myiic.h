#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"

/* EEPROM bit-bang I2C pin mapping.
 * Use the dedicated EE_SCL / EE_SDA pins from main.h so StorageTask does not
 * interfere with RTD_START / RTD_RDY used by ADS1248.
 */

/* IO direction control for EEPROM SDA (PA12). */
#define SDA_IN()  do { GPIOA->MODER &= ~(GPIO_MODER_MODE0 << (12U * 2U)); } while (0)
#define SDA_OUT() do { GPIOA->MODER &= ~(GPIO_MODER_MODE0 << (12U * 2U)); \
                       GPIOA->MODER |=  (1U << (12U * 2U)); } while (0)

/* IO operations on the dedicated EEPROM pins. */
#define IIC_SCL(n)  ( (n) ? HAL_GPIO_WritePin(EE_SCL_GPIO_Port, EE_SCL_Pin, GPIO_PIN_SET) \
                           : HAL_GPIO_WritePin(EE_SCL_GPIO_Port, EE_SCL_Pin, GPIO_PIN_RESET) )
#define IIC_SDA(n)  ( (n) ? HAL_GPIO_WritePin(EE_SDA_GPIO_Port, EE_SDA_Pin, GPIO_PIN_SET) \
                           : HAL_GPIO_WritePin(EE_SDA_GPIO_Port, EE_SDA_Pin, GPIO_PIN_RESET) )
#define WP(n)       ( (n) ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET) \
                           : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET) )
#define READ_SDA    HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin)

/* IIC helper functions */
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
#endif
