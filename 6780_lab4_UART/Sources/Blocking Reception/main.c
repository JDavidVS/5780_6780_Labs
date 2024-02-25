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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* User functions -------------------------------------------------------------*/
void Trans_Character(char ch);
void Trans_String(size_t n, char string[]);



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  SystemClock_Config();

	// ------4.9.1 — Preparing to use the USART------ //
	
	// RCC to enable the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Set the pins to alternate function mode in the MODER register
	// For PC10 -> MODER10 [10] for bit 21 and 20
	GPIOC->MODER |= (1 << 21);
	GPIOC->MODER &= ~(1 << 20);
	// For PC11 -> MODER11 [10] for bit 23 and 22
	GPIOC->MODER |= (1 << 23);
	GPIOC->MODER &= ~(1 << 22);
	// For PC12 -> MODER12 [10] for bit 25 and 24
	GPIOC->MODER |= (1 << 25);
	GPIOC->MODER &= ~(1 << 24);
	
	// Set alternate function registers: 
	// PC10 -> USART3_TX -> AF1
	// PC11 -> USART3_RX -> AF1
	// PC12 -> USART3_CK -> AF1
	// For SEL10 AF1 [0001] for bit [11:8]
	GPIOC->AFR[1] |= (1 << 8);
	GPIOC->AFR[1] &= ~((1 << 9) | (1 << 10) | (1 << 11));
	// For SEL11 AF1 [0001] for bit [15:12]
	GPIOC->AFR[1] |= (1 << 12);
	GPIOC->AFR[1] &= ~((1 << 13) | (1 << 14) | (1 << 15));
	// For SEL12 AF1 [0001] for bit [19:16]
	GPIOC->AFR[1] |= (1 << 16);
	GPIOC->AFR[1] &= ~((1 << 17) | (1 << 18) | (1 << 19));
	
	// ------4.9.2 — Blocking Transmission------ //
	
	// RCC to enable the USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set the Baud rate for communication to be 115200 bits/second
	USART3->BRR = 69;   // Target 69.444... Error of 0.64%
	
	// Enable the transmitter and receiver hardware
	USART3->CR1 |= (1 << 2);	// Receiver Enable
	USART3->CR1 |= (1 << 3);	// Transmitter Enable
	
	// The USART has a peripheral enable/disable bit in its control register
	USART3->CR1 |= (1 << 0);	// USART3 Enable
	
	// ------4.9.4 — Blocking Reception------ //
	
	// Initialize all of the LED pins in the main function
	// RED(PC6) BLUE(PC7) ORANGE(PC8) GREEN(PC9)
	
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

	// Whenever a key is pressed that doesn’t match an LED color, print an error message to the console
	// Create String and calculate its length
	char s[] = "ERROR! The input key does not match an LED color letter";
	// Calculate the number of elements in the array
	size_t n = (sizeof(s) / sizeof(s[0]));
	
	
  while (1)
  {
		// ------4.9.4 — Blocking Reception------ //
		// Check and wait on the USART status flag that indicates the receive (read) register is not empty
		while(!(USART3->ISR & (1 << 5)))	// Triggers if bit 5 (RXNE) is NOT set
		{
			// Empty
		}
		
		// Test the received data and toggle the appropriate LED
		switch(USART3->RDR)
		{
			case (82):		// For 'R'
				GPIOC->ODR |= (1 << 6);
			  GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR &= ~(1 << 9);
				break;
			
			case (114):		// For 'r'
				GPIOC->ODR |= (1 << 6);
			  GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR &= ~(1 << 9);
				break;
			
			case (66):			// For 'B'
				GPIOC->ODR &= ~(1 << 6);
			  GPIOC->ODR |= (1 << 7);
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR &= ~(1 << 9);
				break;
			
			case (98):			// For 'b'
				GPIOC->ODR &= ~(1 << 6);
			  GPIOC->ODR |= (1 << 7);
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR &= ~(1 << 9);
				break;
			
			case (79):			// For 'O'
				GPIOC->ODR &= ~(1 << 6);
			  GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR |= (1 << 8);
				GPIOC->ODR &= ~(1 << 9);
				break;
			
			case (111):			// For 'o'
				GPIOC->ODR &= ~(1 << 6);
			  GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR |= (1 << 8);
				GPIOC->ODR &= ~(1 << 9);
				break;
			
			case (71):			// For 'G'
				GPIOC->ODR &= ~(1 << 6);
			  GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR |= (1 << 9);
				break;
			
			case (103):			// For 'g'
				GPIOC->ODR &= ~(1 << 6);
			  GPIOC->ODR &= ~(1 << 7);
				GPIOC->ODR &= ~(1 << 8);
				GPIOC->ODR |= (1 << 9);
				break;
			
			default:
				Trans_String(n,s);
				
		}
		
  }

}


/**
  * @brief  Transmits a single character on the USART
  * @retval void
  */
void Trans_Character(char ch)
{
	// ------4.9.2 — Blocking Transmission------ //
	
	// Check and wait on the USART status flag that indicates the transmit register is empty
	while(!(USART3->ISR & (1 << 7)))	// Triggers if bit 7 (TXE) is NOT set
	{
		// Empty
	}
	
	// Write the character into the transmit data register
	USART3->TDR = ch;
	
}

/**
  * @brief  Transmits a string on the USART
  * @retval void
  */
void Trans_String(size_t n, char string[])
{
	
	int i;
	// Loop over each element of the array and transmit
	for (i = 0; i < n; i++)
	{
		// Return if the array is terminated
		if (string[i] == 0)
		{
			return;
		}
		
		Trans_Character(string[i]);
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
