/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define Spare_Pin_6_Pin GPIO_PIN_0
#define Spare_Pin_6_GPIO_Port GPIOA
#define USART2_RS485_En_Pin GPIO_PIN_1
#define USART2_RS485_En_GPIO_Port GPIOA
#define ADC1_IN4_Temp_Pin GPIO_PIN_4
#define ADC1_IN4_Temp_GPIO_Port GPIOA
#define ADC1_IN5_Supply_Pin GPIO_PIN_5
#define ADC1_IN5_Supply_GPIO_Port GPIOA
#define ADC1_IN6_Flow_Pin GPIO_PIN_6
#define ADC1_IN6_Flow_GPIO_Port GPIOA
#define Spare_Pin_1_Pin GPIO_PIN_7
#define Spare_Pin_1_GPIO_Port GPIOA
#define Enc_Bit_1_Pin GPIO_PIN_0
#define Enc_Bit_1_GPIO_Port GPIOB
#define Enc_Bit_2_Pin GPIO_PIN_1
#define Enc_Bit_2_GPIO_Port GPIOB
#define Enc_Bit_4_Pin GPIO_PIN_2
#define Enc_Bit_4_GPIO_Port GPIOB
#define LED_Flow_OK_Pin GPIO_PIN_12
#define LED_Flow_OK_GPIO_Port GPIOB
#define LED_GFC_OK_Pin GPIO_PIN_13
#define LED_GFC_OK_GPIO_Port GPIOB
#define LED_Status_Pin GPIO_PIN_14
#define LED_Status_GPIO_Port GPIOB
#define Pump_En_Pin GPIO_PIN_15
#define Pump_En_GPIO_Port GPIOB
#define USART1_RS485_En_Pin GPIO_PIN_8
#define USART1_RS485_En_GPIO_Port GPIOA
#define Dig_In_1_Pin GPIO_PIN_11
#define Dig_In_1_GPIO_Port GPIOA
#define Dig_In_2_Pin GPIO_PIN_12
#define Dig_In_2_GPIO_Port GPIOA
#define Spare_Pin_2_Pin GPIO_PIN_15
#define Spare_Pin_2_GPIO_Port GPIOA
#define Enc_BIt_8_Pin GPIO_PIN_3
#define Enc_BIt_8_GPIO_Port GPIOB
#define USART3_RS485_En_Pin GPIO_PIN_4
#define USART3_RS485_En_GPIO_Port GPIOB
#define Spare_Pin_3_Pin GPIO_PIN_5
#define Spare_Pin_3_GPIO_Port GPIOB
#define Spare_Pin_4_Pin GPIO_PIN_8
#define Spare_Pin_4_GPIO_Port GPIOB
#define Spare_Pin_5_Pin GPIO_PIN_9
#define Spare_Pin_5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern uint32_t g_raw_pressure_sensor;
extern uint32_t g_raw_temperature_sensor; // this variable added by S.Todd 16/01/2023 need to log raw ptx temp value
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
