/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define BLINK_LED_Pin GPIO_PIN_13
#define BLINK_LED_GPIO_Port GPIOC
#define RJ45_LED_L2_Pin GPIO_PIN_1
#define RJ45_LED_L2_GPIO_Port GPIOA
#define USART2_TXEN_Pin GPIO_PIN_4
#define USART2_TXEN_GPIO_Port GPIOA
#define RJ45_LED_R2_Pin GPIO_PIN_5
#define RJ45_LED_R2_GPIO_Port GPIOA
#define RJ45_LED_L3_Pin GPIO_PIN_7
#define RJ45_LED_L3_GPIO_Port GPIOA
#define RJ45_LED_R3_Pin GPIO_PIN_4
#define RJ45_LED_R3_GPIO_Port GPIOC
#define USART3_TXEN_Pin GPIO_PIN_5
#define USART3_TXEN_GPIO_Port GPIOC
#define RJ45_LED_L1_Pin GPIO_PIN_12
#define RJ45_LED_L1_GPIO_Port GPIOB
#define RJ45_LED_R1_Pin GPIO_PIN_13
#define RJ45_LED_R1_GPIO_Port GPIOB
#define E28_AUX_Pin GPIO_PIN_15
#define E28_AUX_GPIO_Port GPIOB
#define E28_M1_Pin GPIO_PIN_8
#define E28_M1_GPIO_Port GPIOC
#define E28_M0_Pin GPIO_PIN_9
#define E28_M0_GPIO_Port GPIOC
#define E28_M2_Pin GPIO_PIN_8
#define E28_M2_GPIO_Port GPIOA
#define AN1_KEY_Pin GPIO_PIN_3
#define AN1_KEY_GPIO_Port GPIOB
#define AN1_LED_Pin GPIO_PIN_4
#define AN1_LED_GPIO_Port GPIOB
#define AN2_KEY_Pin GPIO_PIN_5
#define AN2_KEY_GPIO_Port GPIOB
#define AN2_LED_Pin GPIO_PIN_6
#define AN2_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
