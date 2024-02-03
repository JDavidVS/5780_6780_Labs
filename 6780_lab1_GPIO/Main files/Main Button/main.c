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
	
int main(void) {
	
	SystemClock_Config(); //Configure the system clock
	
	// Lab Assignment: Wirting Basic I/O Code
	
	
	// ------------------ 1.5.1 ----------------- //
	
	
	// 1. Use the RCC to enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// 2. Configure the LED pins to slow-speed, push-pull output mode without pull-up/down resistors
	// The RED and BLUE LEDs are on pins PC6 and PC7
	
	// Set the pins to general-purpose output mode in the MODER register
	// For PC6 -> MODER6 [01] for bit 13 and 12
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	// For PC7 -> MODER7 [01] for bit 15 and 14
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER |= (1 << 14);
	
	// Set the pins to push-pull output type in  the OTYPER register
	// For PC6 -> OTYPER [0] for bit 6
	GPIOC->OTYPER &= ~(1 << 6);
	// For PC7 -> OTYPER [0] for bit 7
	GPIOC->OTYPER &= ~(1 << 7);
	
	// Set the pins to low speed in the OSPEEDR register
	// For PC6 -> OSPEEDR [x0] for bit 13 and 12
	GPIOC->OSPEEDR &= ~((1 << 13) | (1 << 12));
	// For PC7 -> OSPEEDR [x0] for bit 15 and 14
	GPIOC->OSPEEDR &= ~((1 << 15) | (1 << 14));
	
	// Set to no pull-up/down resistors in the PUPDR register
	// FOR PC6 -> PUPDR [00] for bit 13 and 12
	GPIOC->PUPDR &= ~((1 << 13) | (1 << 12));
	// FOR PC7 -> PUPDR [00] for bit 15 and 14
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14));
	
	// 3. Initialize one pin logic high and the other to low
	// PC6 HIGH [1] for bit 6
	GPIOC->ODR |= (1 << 6);
	// PC7 LOW [0] for bit 7
	GPIOC->ODR &= ~(1 << 7);
	
	// ------------------ 1.5.2 ----------------- //
	// 1. B1 USER: User and Wake-Up button connected to the I/O PA0 of the STM32F072RBT6
	
	// 2. Use the RCC to enable the GPIOA peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// 3. Configure the button pin to input mode with the internal pull-down resistor enabled
	
	// Set the pins to input mode in the MODER register
	// For PA0 -> MODER0 [00] for bit 1 and 0
	GPIOA->MODER &= ~(1 << 1);
	GPIOA->MODER &= ~(1 << 0);
	
	// Set the pins to low speed in the OSPEEDR register
	// For PA0 -> OSPEEDR0 [x0] for bit 1 and 0
	GPIOA->OSPEEDR &= ~(1 << 1);
	GPIOA->OSPEEDR &= ~(1 << 0);
	
	// Enable the pull-down resistor in the PUPDR register
	// For PA0 -> PUPDR [10] for bit 1 and 0
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~(1 << 0);
	
	// Debouncer
	uint32_t debouncer = 0;
	
	while (1) {
		
		HAL_Delay(200); // Delay 200ms
		
		debouncer = (debouncer << 1); // Always shift every loop iteration
		
		if (GPIOA->IDR & 1) { // If input signal is set/high
			debouncer |= 0x01; 	// Set lowest bit of bit-vector
			
			// Toggles PC6 and PC7 states
			GPIOC->ODR ^= (1 << 6) | (1 << 7);
		}

		if (debouncer == 0xFFFFFFFF) { 
			// This code triggers repeatedly when button is steady high!
		}
		
		if (debouncer == 0x00000000) {
			// This code triggers repeatedly when button is steady low!
		}
		
		if (debouncer == 0x7FFFFFFF) {
			// This code triggers only once when transitioning to steady high!
		}
	
		// When button is bouncing the bit-vector value is random since bits are set when the button is high and not when it bounces low.	
		
	}
	
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
