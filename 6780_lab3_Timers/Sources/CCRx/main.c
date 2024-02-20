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
	
	// ------3.2 — Configuring Timer Channels to PWM Mode------ //
	// 3.2.1 Enable the timer 3 peripheral (TIM3) in the RCC.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// 3.2.2 The timer’s update period determines the period of the PWM signal; configure the timer to a UEV period related to 800 Hz (T = 1/f)
	// Configure the timer to trigger an update event (UEV) at 800 Hz
  TIM3->PSC = 79;   // Set to 100KHz -> 0.01ms
	TIM3->ARR = 125;  // Count to 1.25ms or 800 Hz
	
	// 3.2.3 Use the Capture/Compare Mode Register 1 (CCMR1) register to configure the output channels to PWM mode
	// The CCMR1 register configures channels 1 & 2, and the CCMR2 register for channels 3 & 4 
	
	// Examine the bit definitions for the CC1S[1:0] and CC2S[1:0] bit fields; ensure that you set the channels to output
	// For CC2S TIM3 -> CCMR1 [00] for bit 9 and 8
	TIM3->CCMR1 &= ~(1 << 9);
	TIM3->CCMR1 &= ~(1 << 8);
	// For CC1S TIM3 -> CCMR1 [00] for bit 1 and 0
	TIM3->CCMR1 &= ~(1 << 1);
	TIM3->CCMR1 &= ~(1 << 0);
	
	// Examine the bit definitions for the OC1M[2:0] bit field; set output channel 1 to PWM Mode 2
	// For OC1M TIM3 -> CCMR1 [111] for bit 6, 5, 4
	TIM3->CCMR1 |= (1 << 6);
	TIM3->CCMR1 |= (1 << 5);
	TIM3->CCMR1 |= (1 << 4);
	
	// Use the OC2M[2:0] bit field to set channel 2 to PWM Mode 1
	// For OC2M TIM3 -> CCMR1 [110] for bit 14, 13, 12 
	TIM3->CCMR1 |= (1 << 14);
	TIM3->CCMR1 |= (1 << 13);
	TIM3->CCMR1 &= ~(1 << 12);
	
	// Enable the output compare preload for both channels
	// For OC1PE TIM3 -> CCMR1 [1] for bit 3
	TIM3->CCMR1 |= (1 << 3);
	// For OC2PE TIM3 -> CCMR1 [1] for bit 11
	TIM3->CCMR1 |= (1 << 11);
	
	// 3.2.4 Set the output enable bits for channels 1 & 2 in the CCER register
	// For CCE1 TIM3-> CCER [1] for bit 0
	TIM3->CCER |= (1 << 0);
	// For CCE2 TIM3 -> CCER [1] for bit 4
	TIM3->CCER |= (1 << 4);
	
	// 3.2.5 Set the capture/compare registers (CCRx) for both channels to 20% of your ARR value
	TIM3->CCR1 = 25;
	TIM3->CCR2 = 25;
	
	TIM3->CR1 |= (1 << 0);
	
	// ------3.3 — Configuring Pin Alternate Functions------ // 
	// 3.3.1 Look up the alternate functions of the red (PC6) and blue (PC7) LEDs
	
	// 3.3.2 Configure the LED pins to alternate function mode, and select the appropriate function number in alternate function registers
	//Set the pins to alternate function mode in the MODER register
	// For PC6 -> MODER6 [10] for bit 13 and 12
	GPIOC->MODER |= (1 << 13);
	GPIOC->MODER &= ~(1 << 12);
	// For PC7 -> MODER7 [10] for bit 15 and 14
	GPIOC->MODER |= (1 << 15);
	GPIOC->MODER &= ~(1 << 14);
	
	// Set alternate function registers: PC6 -> TIM3_CH1 -> AF0 and PC7 -> TIM3_CH2 -> AF0
	// For SEL6 AF0 [0000] for bit [27:24]
	GPIOC->AFR[0] &= ~((1 << 27) | (1 << 26) | (1 << 25) | (1 << 24));
	// For SEL7 AF0 [0000] for bit [31:28]
	GPIOC->AFR[0] &= ~((1 << 31) | (1 << 30) | (1 << 29) | (1 << 28));
	
	
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
