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


void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  SystemClock_Config();
	
	// Initialize the LED pins to output
	// RCC to enable the GPIOC, ADC1 peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// 				LED SETUP 					//
	//														//
	// Initialize all of the LED pins in the main function
	// RED(PC6) BLUE(PC7) ORANGE(PC8) GREEN(PC9)
	
	//Set the pins to general-purpose output mode in the MODER register
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->MODER |= (1 << 18);
	//Set the pins to push-pull output type in  the OTYPER register
	GPIOC->OTYPER &= ~(1 << 6);
	GPIOC->OTYPER &= ~(1 << 7);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OTYPER &= ~(1 << 9);
	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 13) | (1 << 12));
	GPIOC->OSPEEDR &= ~((1 << 15) | (1 << 14));
	GPIOC->OSPEEDR &= ~((1 << 17) | (1 << 16));
	GPIOC->OSPEEDR &= ~((1 << 19) | (1 << 18));
	// Set to no pull-up/down resistors in the PUPDR register
	GPIOC->PUPDR &= ~((1 << 13) | (1 << 12));
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14));
	GPIOC->PUPDR &= ~((1 << 17) | (1 << 16));
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18));
	
	// Select a GPIO pin to use as the ADC input
	//Set the pins to analog mode in the MODER register
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER |= (1 << 1);
	
	// Configure the ADC to 8-bit resolution, continuous conversion mode, hardware triggers disabled (software trigger only)
	// 8-bit resolution
	ADC1->CFGR1 &= ~(1 << 4);
	ADC1->CFGR1 |= (1 << 3);
	// Continuous conversion mode
	ADC1->CFGR1 |= (1 << 13);
	// Hardware triggers disabled
	ADC1->CFGR1 &= ~(1 << 10);
	ADC1->CFGR1 &= ~(1 << 11);
	
	// Select/enable the input pin’s channel for ADC conversion
	ADC1->CHSELR |= (1 << 10);
	
	// Perform a self-calibration, enable, and start the ADC
	//
	// ADC calibration
	//
	// Ensure that ADEN = 0 and DMAEN = 0.
	if ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		// Clear ADEN by setting ADDIS
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0){};
	// Clear DMAEN
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	// Launch the calibration by setting ADCAL
	ADC1->CR |= ADC_CR_ADCAL;
	// Wait until ADCAL=0
	while ((ADC1->CR & ADC_CR_ADCAL) != 0){};
	
	//
	// ADC enable sequence
  //
  // Ensure that ADRDY = 0
	if ((ADC1->ISR |= ADC_ISR_ADRDY) == 0)
	{
		// Clear ADRDY
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	// Enable the ADC
	ADC1->CR |= ADC_CR_ADEN;
	// Wait until ADC ready
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0){};
		
	// Start the ADC conversion
	ADC1->CR |= ADC_CR_ADSTART;
		
	// Variables
	int16_t OUTPUT = 0;
		
  while (1)
  {
    // In the main application loop, read the ADC data register and turn on/off LEDs depending on the value
		OUTPUT = ADC1->DR;
		
		
		// LED Controller
		//
		// RED(PC6) = 0 to 64		
		// BLUE(PC7) = 65 to 128
		// ORANGE(PC8) = 129 to 192 	
		// GREEN(PC9) = 193 to 256
		
		if (OUTPUT <= 64)
		{
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
		
		if ((OUTPUT >= 65) && (OUTPUT <= 128))
		{
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR |= (1 << 7);
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
		
		if ((OUTPUT >= 129) && (OUTPUT <= 192))
		{
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
		
		else if (OUTPUT > 193)
		{
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR |= (1 << 9);
		}
			
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
