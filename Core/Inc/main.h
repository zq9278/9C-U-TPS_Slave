/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define UART_RX_BUF_SIZE 256
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint8_t buffer[UART_RX_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer_t;

typedef struct {
    uint8_t frame_id[2];      // ֡ ID
    uint8_t *payload;         // ����ָ��
    uint8_t payload_len;      // ���ݳ���
} ProtocolFrame_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Fan1_Pin GPIO_PIN_1
#define Fan1_GPIO_Port GPIOC
#define Fan2_Pin GPIO_PIN_2
#define Fan2_GPIO_Port GPIOC
#define Fan3_Pin GPIO_PIN_3
#define Fan3_GPIO_Port GPIOC
#define AirValve1_Pin GPIO_PIN_0
#define AirValve1_GPIO_Port GPIOA
#define AirValve2_Pin GPIO_PIN_1
#define AirValve2_GPIO_Port GPIOA
#define AirPump1_Pin GPIO_PIN_2
#define AirPump1_GPIO_Port GPIOA
#define Heat1_PWM_Pin GPIO_PIN_4
#define Heat1_PWM_GPIO_Port GPIOA
#define Heat1_NTC1_V_Pin GPIO_PIN_5
#define Heat1_NTC1_V_GPIO_Port GPIOA
#define OTP1_RESET_Pin GPIO_PIN_6
#define OTP1_RESET_GPIO_Port GPIOA
#define Heat2_PWM_Pin GPIO_PIN_7
#define Heat2_PWM_GPIO_Port GPIOA
#define Heat2_NTC1_V_Pin GPIO_PIN_0
#define Heat2_NTC1_V_GPIO_Port GPIOB
#define OTP2_RESET_Pin GPIO_PIN_1
#define OTP2_RESET_GPIO_Port GPIOB
#define RTD_DOUT_Pin GPIO_PIN_2
#define RTD_DOUT_GPIO_Port GPIOB
#define RTD_SCK_Pin GPIO_PIN_10
#define RTD_SCK_GPIO_Port GPIOB
#define RTD_DIN_Pin GPIO_PIN_11
#define RTD_DIN_GPIO_Port GPIOB
#define RTD_CS_Pin GPIO_PIN_12
#define RTD_CS_GPIO_Port GPIOB
#define Heat2_SCL_Pin GPIO_PIN_13
#define Heat2_SCL_GPIO_Port GPIOB
#define Heat2_SDA_Pin GPIO_PIN_14
#define Heat2_SDA_GPIO_Port GPIOB
#define RTD_START_Pin GPIO_PIN_15
#define RTD_START_GPIO_Port GPIOB
#define RTD_RDY_Pin GPIO_PIN_8
#define RTD_RDY_GPIO_Port GPIOA
#define Heat1_SCL_Pin GPIO_PIN_9
#define Heat1_SCL_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_7
#define LED0_GPIO_Port GPIOC
#define Heat1_SDA_Pin GPIO_PIN_10
#define Heat1_SDA_GPIO_Port GPIOA
#define EE_SCL_Pin GPIO_PIN_11
#define EE_SCL_GPIO_Port GPIOA
#define EE_SDA_Pin GPIO_PIN_12
#define EE_SDA_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_15
#define OLED_SDA_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOC
#define WaterValve1_Pin GPIO_PIN_9
#define WaterValve1_GPIO_Port GPIOC
#define WaterValve2_Pin GPIO_PIN_0
#define WaterValve2_GPIO_Port GPIOD
#define WaterValve3_Pin GPIO_PIN_1
#define WaterValve3_GPIO_Port GPIOD
#define WaterValve4_Pin GPIO_PIN_2
#define WaterValve4_GPIO_Port GPIOD
#define WaterValve5_Pin GPIO_PIN_3
#define WaterValve5_GPIO_Port GPIOD
#define AirPump3_Pin GPIO_PIN_4
#define AirPump3_GPIO_Port GPIOD
#define TS1_Pin GPIO_PIN_5
#define TS1_GPIO_Port GPIOB
#define SW_CNT_Pin GPIO_PIN_6
#define SW_CNT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Fan1_Pin GPIO_PIN_1
#define Fan1_GPIO_Port GPIOC
#define Fan2_Pin GPIO_PIN_2
#define Fan2_GPIO_Port GPIOC
#define Fan3_Pin GPIO_PIN_3
#define Fan3_GPIO_Port GPIOC
#define AirValve1_Pin GPIO_PIN_0
#define AirValve1_GPIO_Port GPIOA
#define AirValve2_Pin GPIO_PIN_1
#define AirValve2_GPIO_Port GPIOA
#define AirPump1_Pin GPIO_PIN_2
#define AirPump1_GPIO_Port GPIOA
#define Heat1_PWM_Pin GPIO_PIN_4
#define Heat1_PWM_GPIO_Port GPIOA
#define Heat1_NTC1_V_Pin GPIO_PIN_5
#define Heat1_NTC1_V_GPIO_Port GPIOA
#define OTP1_RESET_Pin GPIO_PIN_6
#define OTP1_RESET_GPIO_Port GPIOA
#define Heat2_PWM_Pin GPIO_PIN_7
#define Heat2_PWM_GPIO_Port GPIOA
#define Heat2_NTC1_V_Pin GPIO_PIN_0
#define Heat2_NTC1_V_GPIO_Port GPIOB
#define OTP2_RESET_Pin GPIO_PIN_1
#define OTP2_RESET_GPIO_Port GPIOB
#define RTD_DOUT_Pin GPIO_PIN_2
#define RTD_DOUT_GPIO_Port GPIOB
#define RTD_SCK_Pin GPIO_PIN_10
#define RTD_SCK_GPIO_Port GPIOB
#define RTD_DIN_Pin GPIO_PIN_11
#define RTD_DIN_GPIO_Port GPIOB
#define RTD_CS_Pin GPIO_PIN_12
#define RTD_CS_GPIO_Port GPIOB
#define Heat2_SCL_Pin GPIO_PIN_13
#define Heat2_SCL_GPIO_Port GPIOB
#define Heat2_SDA_Pin GPIO_PIN_14
#define Heat2_SDA_GPIO_Port GPIOB
#define RTD_START_Pin GPIO_PIN_15
#define RTD_START_GPIO_Port GPIOB
#define RTD_RDY_Pin GPIO_PIN_8
#define RTD_RDY_GPIO_Port GPIOA
#define Heat1_SCL_Pin GPIO_PIN_9
#define Heat1_SCL_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_7
#define LED0_GPIO_Port GPIOC
#define Heat1_SDA_Pin GPIO_PIN_10
#define Heat1_SDA_GPIO_Port GPIOA
#define EE_SCL_Pin GPIO_PIN_11
#define EE_SCL_GPIO_Port GPIOA
#define EE_SDA_Pin GPIO_PIN_12
#define EE_SDA_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_15
#define OLED_SDA_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOC
#define WaterValve1_Pin GPIO_PIN_9
#define WaterValve1_GPIO_Port GPIOC
#define WaterValve2_Pin GPIO_PIN_0
#define WaterValve2_GPIO_Port GPIOD
#define WaterValve3_Pin GPIO_PIN_1
#define WaterValve3_GPIO_Port GPIOD
#define WaterValve4_Pin GPIO_PIN_2
#define WaterValve4_GPIO_Port GPIOD
#define WaterValve5_Pin GPIO_PIN_3
#define WaterValve5_GPIO_Port GPIOD
#define AirPump3_Pin GPIO_PIN_4
#define AirPump3_GPIO_Port GPIOD
#define WaterPump1_Pin GPIO_PIN_3
#define WaterPump1_GPIO_Port GPIOB
#define WS1_Pin GPIO_PIN_4
#define WS1_GPIO_Port GPIOB
#define TS1_Pin GPIO_PIN_5
#define TS1_GPIO_Port GPIOB
#define SW_CNT_Pin GPIO_PIN_6
#define SW_CNT_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
