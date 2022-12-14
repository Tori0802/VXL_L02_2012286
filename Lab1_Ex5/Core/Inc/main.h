/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LED_LRED_Pin GPIO_PIN_5
#define LED_LRED_GPIO_Port GPIOA
#define LED_LYELLOW_Pin GPIO_PIN_6
#define LED_LYELLOW_GPIO_Port GPIOA
#define LED_LGREEN_Pin GPIO_PIN_7
#define LED_LGREEN_GPIO_Port GPIOA
#define LED_P1_Pin GPIO_PIN_0
#define LED_P1_GPIO_Port GPIOB
#define LED_P2_Pin GPIO_PIN_1
#define LED_P2_GPIO_Port GPIOB
#define LED_P3_Pin GPIO_PIN_2
#define LED_P3_GPIO_Port GPIOB
#define LED_L4_Pin GPIO_PIN_10
#define LED_L4_GPIO_Port GPIOB
#define LED_L5_Pin GPIO_PIN_11
#define LED_L5_GPIO_Port GPIOB
#define LED_L6_Pin GPIO_PIN_12
#define LED_L6_GPIO_Port GPIOB
#define LED_L7_Pin GPIO_PIN_13
#define LED_L7_GPIO_Port GPIOB
#define LED_PRED_Pin GPIO_PIN_8
#define LED_PRED_GPIO_Port GPIOA
#define LED_PYELLOW_Pin GPIO_PIN_9
#define LED_PYELLOW_GPIO_Port GPIOA
#define LED_PGREEN_Pin GPIO_PIN_10
#define LED_PGREEN_GPIO_Port GPIOA
#define LED_P4_Pin GPIO_PIN_3
#define LED_P4_GPIO_Port GPIOB
#define LED_P5_Pin GPIO_PIN_4
#define LED_P5_GPIO_Port GPIOB
#define LED_P6_Pin GPIO_PIN_5
#define LED_P6_GPIO_Port GPIOB
#define LED_P7_Pin GPIO_PIN_6
#define LED_P7_GPIO_Port GPIOB
#define LED_L1_Pin GPIO_PIN_7
#define LED_L1_GPIO_Port GPIOB
#define LED_L2_Pin GPIO_PIN_8
#define LED_L2_GPIO_Port GPIOB
#define LED_L3_Pin GPIO_PIN_9
#define LED_L3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
