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
	
	// ------2.1 — Modifying an Existing Interrupt:-------- //
	
	// Initialize all of the LED pins in the main function
	
	// RED(PC6) BLUE(PC7) ORANGE(PC8) GREEN(PC8)
	// RCC to enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Set the pins to general-purpose output mode in the MODER register
	// For PC6 -> MODER6 [01] for bit 13 and 12
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	// For PC7 -> MODER7 [01] for bit 15 and 14
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER |= (1 << 14);
	// For PC8 -> MODER8 [01] for bit 17 and 16
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	// For PC9 -> MODER9 [01] for bit 19 and 18
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->MODER |= (1 << 18);
	
	//Set the pins to push-pull output type in  the OTYPER register
	// For PC6 -> OTYPER [0] for bit 6
	GPIOC->OTYPER &= ~(1 << 6);
	// For PC7 -> OTYPER [0] for bit 7
	GPIOC->OTYPER &= ~(1 << 7);
	// For PC8 -> OTYPER [0] for bit 8
	GPIOC->OTYPER &= ~(1 << 8);
	// For PC9 -> OTYPER [0] for bit 9
	GPIOC->OTYPER &= ~(1 << 9);
	
	// Set the pins to low speed in the OSPEEDR register
	// For PC6 -> OSPEEDR [x0] for bit 13 and 12
	GPIOC->OSPEEDR &= ~((1 << 13) | (1 << 12));
	// For PC7 -> OSPEEDR [x0] for bit 15 and 14
	GPIOC->OSPEEDR &= ~((1 << 15) | (1 << 14));
	// For PC8 -> OSPEEDR [x0] for bit 17 and 16
	GPIOC->OSPEEDR &= ~((1 << 17) | (1 << 16));
	// For PC9 -> OSPEEDR [x0] for bit 19 and 18
	GPIOC->OSPEEDR &= ~((1 << 19) | (1 << 18));
	
	// Set to no pull-up/down resistors in the PUPDR register
	// FOR PC6 -> PUPDR [00] for bit 13 and 12
	GPIOC->PUPDR &= ~((1 << 13) | (1 << 12));
	// FOR PC7 -> PUPDR [00] for bit 15 and 14
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14));
	// FOR PC8 -> PUPDR [00] for bit 17 and 16
	GPIOC->PUPDR &= ~((1 << 17) | (1 << 16));
	// FOR PC9 -> PUPDR [00] for bit 19 and 18
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18));
	
	
	// Set the GREEN(PC9) LED HIGH
	// PC9 HIGH [1] for bit 9
	GPIOC->ODR |= (1 << 9);
	// Set the BLUE(PC7) LED LOW
	// PC7 LOW [0] for bit 7
	GPIOC->ODR &= ~(1 << 7);
	
	
	// ------2.2 — Configuring the EXTI:-------- //
	
	// 2.2.1. Configure the button pin (PA0) to input-mode at low-speed, with the internal pull-down resistor enabled.
	
	// RCC to enable the GPIOA peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
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
	
	// 2.2.2. Pin PA0 connects to the EXTI input line 0 (EXTI0)
	
	
		// ------2.3 — Setting the SYSCFG Pin Multiplexer.:-------- //
		
		// 2.3.1 Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		
		// 2.3.2 Determine which SYSCFG multiplexer can route PA0 to the EXTI peripheral.
		// SYSCFG_EXTICRx
		
		// 2.3.3 Each of the EXTICRx registers control multiple pin multiplexers. Find which register contains
		// the configuration bits for the required multiplexer.
		// 0x0000
		
		// 2.3.4 Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0)
		SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
		
		
	// 2.2.3. Enable/unmask interrupt generation on EXTI input line 0 (EXTI0)
	// For EXTI0 -> IMR0 [1]  for bit 0
	EXTI->IMR |= (1 << 0);
	
	// 2.2.4. Configure the EXTI input line 0 to have a rising-edge trigger
	// For EXIT0 -> RTSR0 [1] for bit 0
	EXTI->RTSR |= (1 << 0);
	
	
	// ------2.4 — Enable and Set Priority of the EXTI Interrupt:-------- //
	
	// 2.4.2 Select the entry that references the EXTI input line 0
	// EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupt*/
	
	// 2.4.3 Enable the selected EXTI interrupt by passing its defined name to the NVIC_EnableIRQ() function. (located in core_cm0.h)
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	// 2.4.4 Set the priority for the interrupt to 1 (high-priority) with the NVIC_SetPriority() function.
	NVIC_SetPriority(EXTI0_1_IRQn,1);   /// Set to 1
	
	
	// ------2.7 — Exploring Interrupt Priorities:-------- //
	NVIC_SetPriority(SysTick_IRQn,3);		
	
	
	
  while (1)
  {
    // 2.1.3. Toggle the RED LED (PC6) with a moderately-slow delay (400-600ms) in the infinite loop.
		
		HAL_Delay(500); // Delay 500ms
		
		// PC6 HIGH [1] for bit 6
		GPIOC->ODR |= (1 << 6);
		
		HAL_Delay(500); // Delay 500ms
		
		// PC6 LOW [1] for bit 6
		GPIOC->ODR &= ~(1 << 6);
		
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
