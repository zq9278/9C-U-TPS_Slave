#ifndef _SYS_H
#define _SYS_H
#include "stm32G0xx.h"
#include "core_cm0plus.h"
#include "stm32G0xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "types_compat.h"


//0,��֧��os
//1,֧��os
#define SYSTEM_SUPPORT_OS		0		//����ϵͳ�ļ����Ƿ�֧��OS	
#define DEBUG		1//DEBUGģʽ
///////////////////////////////////////////////////////////////////////////////////
//����һЩ���õ��������Ͷ̹ؼ��� 
//typedef int64_t  s64;
//typedef int32_t  s32;
//typedef int16_t s16;
//typedef int8_t  s8;

//typedef const int32_t sc32;  
//typedef const int16_t sc16;  
//typedef const int8_t sc8;  

//typedef __IO int32_t  vs32;
//typedef __IO int16_t  vs16;
//typedef __IO int8_t   vs8;

//typedef __I int32_t vsc32;  
//typedef __I int16_t vsc16; 
//typedef __I int8_t vsc8;   

//typedef uint32_t  u32;
//typedef uint16_t u16;
//typedef uint8_t  u8;

//typedef const uint32_t uc32;  
//typedef const uint16_t uc16;  
//typedef const uint8_t uc8; 

//typedef __IO uint32_t  vu32;
//typedef __IO uint16_t vu16;
//typedef __IO uint8_t  vu8;

//typedef __I uint32_t vuc32;  
//typedef __I uint16_t vuc16; 
//typedef __I uint8_t vuc8;  

#define ON	1
#define OFF	0
#define ForceSen	4227

#define Write_Through() (*(__IO uint32_t*)0XE000EF9C=1UL<<2) //Cache͸дģʽ

#define TMC_ENN(n)		(n?HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_RESET))
#define TMC_CSN(n)		(n?HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET))

#define CHG_CE(n)			(n?HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_RESET))

#define LED0(n)				(n?HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET))
#define LED1(n)				(n?HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET))
#define LED0Toggle		HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin)
#define LED1Toggle		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin)

#define KEY1					HAL_GPIO_ReadPin(SW_CNT_GPIO_Port,SW_CNT_Pin)
#define PWR_SW				HAL_GPIO_ReadPin(PWR_SENSE_GPIO_Port,PWR_SENSE_Pin)

#define BQ25895Address	0xd4
#define BQ27441Address	0xaa
#define TMP114Address		0x90


////λ������,ʵ��51���Ƶ�GPIO���ƹ���
////����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
////IO�ڲ����궨��
//#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
//#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
//#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
////IO�ڵ�ַӳ��
//#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
//#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
//#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
//#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
//#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
//#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
//#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
//#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
//#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

//#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
//#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
//#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
//#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
//#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
//#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
//#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
//#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
//#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
////IO�ڲ���,ֻ�Ե�һ��IO��!
////ȷ��n��ֵС��16!
//#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
//#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

//#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
//#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

//#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
//#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

//#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
//#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

//#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
//#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

//#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
//#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

//#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
//#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

//#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
//#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

//#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
//#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

void Cache_Enable(void);                                    //ʹ��STM32H7��L1-Cahce
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq); //����ϵͳʱ��
u8 Get_ICahceSta(void);//�ж�I_Cache�Ƿ��
u8 Get_DCahceSta(void);//�ж�I_Dache�Ƿ��

/*������ؼĴ�����ַ*/
#define TargetAdd						0x20
#define Motor1PositionAdd		0x15
//#define VPPnom							1360//5700  1621
#define VPPnom							1360//5700  1621

#if defined(__clang__) //ʹ��V6������(clang)
void __attribute__((noinline)) WFI_SET(void);
void __attribute__((noinline)) INTX_DISABLE(void);
void __attribute__((noinline)) INTX_ENABLE(void);
void __attribute__((noinline)) MSR_MSP(u32 addr);
#elif defined (__CC_ARM)    //ʹ��V5������(ARMCC)
//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 
#endif


#endif

