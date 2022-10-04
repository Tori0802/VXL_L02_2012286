/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Khoi tao cac bien dem "counterL" va "counterP" co gia tri khoi tao ban dau lan luot la 5 va 3 tuong ung cho truong hop ban dau la luong L (Landscape) den RED va luong P (Portrait) den GREEN */
  /* Khoi tao bien "change" co gia tri ban dau la 0. Bien nay duoc dung de thong bao thay doi tu den GREEN sang den YELLOW */
  /* Khoi tao bien "sw" co gia tri ban dau la "P" (Portrait) de chi trang thai luong dang chay cua 4-way traffic light */
  int counterL = 5, counterP = 3, change = 0;
  char sw = 'P';
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  counterL--; counterP--;
	  /* Truong hop "sw" = 'P' (Luong P duoc chay, luong L dung den RED) */
	  if (sw == 'P') {
		  HAL_GPIO_WritePin(LED_LRED_GPIO_Port , LED_LRED_Pin , GPIO_PIN_RESET); /* Thiet lap den RED cho luong L */
		  HAL_GPIO_WritePin(LED_LYELLOW_GPIO_Port , LED_LYELLOW_Pin , GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_LGREEN_GPIO_Port , LED_LGREEN_Pin , GPIO_PIN_SET);
		  /* Trong khoang thoi gian 6 giay luong L dang den RED, thi o luong P ta se thiet lap 4 giay dau den GREEN va 2 giay sau den YELLOW */
		  if (counterL >= 0) {
			  if (counterP >= 0) {
				  /* "change" = 0 thi den GREEN, "change" = 1 thi den YELLOW */
				  if (change == 0) {
					  HAL_GPIO_WritePin(LED_PRED_GPIO_Port , LED_PRED_Pin , GPIO_PIN_SET); /* Thiet lap den GREEN cho luong P */
				  	  HAL_GPIO_WritePin(LED_PYELLOW_GPIO_Port , LED_PYELLOW_Pin , GPIO_PIN_SET);
				  	  HAL_GPIO_WritePin(LED_PGREEN_GPIO_Port , LED_PGREEN_Pin , GPIO_PIN_RESET);
				  }
				  else {
					  HAL_GPIO_WritePin(LED_PRED_GPIO_Port , LED_PRED_Pin , GPIO_PIN_SET); /* Thiet lap den YELLOW cho luong P */
					  HAL_GPIO_WritePin(LED_PYELLOW_GPIO_Port , LED_PYELLOW_Pin , GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(LED_PGREEN_GPIO_Port , LED_PGREEN_Pin , GPIO_PIN_SET);
				  }
			  	}
			  else { /* Chuyen doi trang thai GREEN sang YELLOW o luong P */
				  change = 1;
				  counterP = 1;
			  }
		  }
		  else { /* Thiet lap lai cac thong so de chuyen luong giao thong sang luong L duoc chay, luong P dung lai (sw = L) */
			  sw = 'L';
			  counterL = 3;
			  counterP = 5;
			  change = 0;
		  }
	  }

	  /* Truong hop "sw" = 'L' (Luong L duoc chay, luong P dung den RED) */
	  else {
		  HAL_GPIO_WritePin(LED_PRED_GPIO_Port , LED_PRED_Pin , GPIO_PIN_RESET); /* Thiet lap den RED cho luong P */
		  HAL_GPIO_WritePin(LED_PYELLOW_GPIO_Port , LED_PYELLOW_Pin , GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_PGREEN_GPIO_Port , LED_PGREEN_Pin , GPIO_PIN_SET);
		  /* Trong khoang thoi gian 6 giay luong P dang den RED, thi o luong L ta se thiet lap 4 giay dau den GREEN va 2 giay sau den YELLOW */
		  if (counterP >= 0) {
			  if (counterL >= 0) {
				  /* "change" = 0 thi den GREEN, "change" = 1 thi den YELLOW */
				  if (change == 0) {
					  HAL_GPIO_WritePin(LED_LRED_GPIO_Port , LED_LRED_Pin , GPIO_PIN_SET); /* Thiet lap den GREEN cho luong L */
				  	  HAL_GPIO_WritePin(LED_LYELLOW_GPIO_Port , LED_LYELLOW_Pin , GPIO_PIN_SET);
				  	  HAL_GPIO_WritePin(LED_LGREEN_GPIO_Port , LED_LGREEN_Pin , GPIO_PIN_RESET);
				  }
				  else {
					  HAL_GPIO_WritePin(LED_LRED_GPIO_Port , LED_LRED_Pin , GPIO_PIN_SET); /* Thiet lap den YELLOW cho luong L */
					  HAL_GPIO_WritePin(LED_LYELLOW_GPIO_Port , LED_LYELLOW_Pin , GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(LED_LGREEN_GPIO_Port , LED_LGREEN_Pin , GPIO_PIN_SET);
				  }
			  	}
			  else { /* Chuyen doi trang thai GREEN sang YELLOW o luong L */
				  change = 1;
				  counterL = 1;
			  }
		  }
		  else { /* Thiet lap lai cac thong so de chuyen luong giao thong sang luong P duoc chay, luong L dung lai (sw = P) */
			  sw = 'P';
			  counterP = 3;
			  counterL = 5;
			  change = 0;
		  }
	  }
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_LRED_Pin|LED_LYELLOW_Pin|LED_LGREEN_Pin|LED_PRED_Pin
                          |LED_PYELLOW_Pin|LED_PGREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_LRED_Pin LED_LYELLOW_Pin LED_LGREEN_Pin LED_PRED_Pin
                           LED_PYELLOW_Pin LED_PGREEN_Pin */
  GPIO_InitStruct.Pin = LED_LRED_Pin|LED_LYELLOW_Pin|LED_LGREEN_Pin|LED_PRED_Pin
                          |LED_PYELLOW_Pin|LED_PGREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
