/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
  SystemClock_Config();

	// ------3.1 — Using Timer Interrupts------ //
	
	// 3.1.6 Initialize the LED pins in the main function
	// ORANGE(PC8) GREEN(PC9)
	// RCC to enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Set the pins to general-purpose output mode in the MODER register
	// For PC8 -> MODER8 [01] for bit 17 and 16
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	// For PC9 -> MODER9 [01] for bit 19 and 18
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->MODER |= (1 << 18);
	
	//Set the pins to push-pull output type in  the OTYPER register
	// For PC8 -> OTYPER [0] for bit 8
	GPIOC->OTYPER &= ~(1 << 8);
	// For PC9 -> OTYPER [0] for bit 9
	GPIOC->OTYPER &= ~(1 << 9);
	
	// Set the pins to low speed in the OSPEEDR register
	// For PC8 -> OSPEEDR [x0] for bit 17 and 16
	GPIOC->OSPEEDR &= ~((1 << 17) | (1 << 16));
	// For PC9 -> OSPEEDR [x0] for bit 19 and 18
	GPIOC->OSPEEDR &= ~((1 << 19) | (1 << 18));
	
	// Set to no pull-up/down resistors in the PUPDR register
	// FOR PC8 -> PUPDR [00] for bit 17 and 16
	GPIOC->PUPDR &= ~((1 << 17) | (1 << 16));
	// FOR PC9 -> PUPDR [00] for bit 19 and 18
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18));
	
	// Set the ORANGE(PC8) LED HIGH
	// PC8 HIGH [1] for bit 8
	GPIOC->ODR |= (1 << 8);
	// Set the GREEN(PC9) LED LOW
	// PC9 LOW [0] for bit 9
	GPIOC->ODR &= ~(1 << 9);
	
	// 3.1.1 Enable TIM2 peripheral in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// 3.1.2 Configure the timer to trigger an update event (UEV) at 4 Hz
	TIM2->PSC = 7999;   // Set to 1KHz
	TIM2->ARR = 250;		// Count to 250ms or 4 Hz

	// 3.1.3 Configure the timer to generate an interrupt on the UEV event
	// Use the DMA/Interrupt Enable Register (DIER) to enable the update interrupt
	// For TIM2 -> DIER0 [1] for bit 0
	TIM2->DIER |= (1 << 0);
	
	// 3.1.4 Configure and enable/start the timer
	// For TIM2 -> CR10 [1] for bit 0
	TIM2->CR1 |= (1 << 0);
	
	// 3.1.5 Set up the timer’s interrupt handler, and enable in the NVIC
	// Enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	
	
	
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
