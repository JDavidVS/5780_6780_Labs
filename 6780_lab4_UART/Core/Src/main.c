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

/* Global Variables -----------------------------------------------------------*/
uint32_t Rec_Reg;
int Flag_New_Data;


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
	
	// ------4.9.5 — Interrupt-Based Reception------ //
	// Enable the receive register not empty interrupt
	USART3->CR1 |= (1 << 5); 	// RXNEIE Enable
	
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
	
	// ------4.9.5 — Interrupt-Based Reception------ //
	
	// Enable and set the USART interrupt priority in the NVIC
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);
	
	// Whenever a key is pressed that doesn’t match an LED color, print an error message to the console
	// Create String and calculate its length
	char msg1[] = "ERROR! Unknown character\r\n\n";
	char msg2[] = "Enter Command:\r\n";
	// Calculate the number of elements in the array
	size_t n1 = (sizeof(msg1) / sizeof(msg1[0]));
	size_t n2 = (sizeof(msg2) / sizeof(msg2[0]));
	
	// Command LED MSGs
	// RED
	char msg3[] = "RED OFF\r\n\n";
	size_t n3 = (sizeof(msg3) / sizeof(msg3[0]));
	char msg4[] = "RED ON\r\n\n";
	size_t n4 = (sizeof(msg4) / sizeof(msg4[0]));
	char msg5[] = "RED TOGGLE\r\n\n";
	size_t n5 = (sizeof(msg5) / sizeof(msg5[0]));
	// BLUE
	char msg6[] = "BLUE OFF\r\n\n";
	size_t n6 = (sizeof(msg6) / sizeof(msg6[0]));
	char msg7[] = "BLUE ON\r\n\n";
	size_t n7 = (sizeof(msg7) / sizeof(msg7[0]));
	char msg8[] = "BLUE TOGGLE\r\n\n";
	size_t n8 = (sizeof(msg8) / sizeof(msg8[0]));
	// ORANGE
	char msg9[] = "ORANGE OFF\r\n\n";
	size_t n9 = (sizeof(msg9) / sizeof(msg9[0]));
	char msg10[] = "ORANGE ON\r\n\n";
	size_t n10 = (sizeof(msg10) / sizeof(msg10[0]));
	char msg11[] = "ORANGE TOGGLE\r\n\n";
	size_t n11 = (sizeof(msg11) / sizeof(msg11[0]));
	// GREEN
	char msg12[] = "GREEN OFF\r\n\n";
	size_t n12 = (sizeof(msg12) / sizeof(msg12[0]));
	char msg13[] = "GREEN ON\r\n\n";
	size_t n13 = (sizeof(msg13) / sizeof(msg13[0]));
	char msg14[] = "GREEN TOGGLE\r\n\n";
	size_t n14 = (sizeof(msg14) / sizeof(msg14[0]));
	
	int countMsg = 0;
	int led;
	
  while (1)
  {
		// ------4.9.4 — Blocking Reception------ //
		// Check and wait on the USART status flag that indicates the receive (read) register is not empty
		while(!(USART3->ISR & (1 << 5)))	// Triggers if bit 5 (RXNE) is NOT set
		{
			// Empty
		}
		
		Trans_String(n2,msg2);
		
		// IF condition for first LED key letter
		if (Flag_New_Data == 1 && countMsg == 0)
		{
			// IF 'r' OR 'R'
			if (Rec_Reg == 82 || Rec_Reg == 114)
			{
				Flag_New_Data = 0;
				countMsg = 1;
				led = 6;
			}
			// IF 'b' OR 'B'
			else if (Rec_Reg == 66 || Rec_Reg == 98)
			{
				Flag_New_Data = 0;
				countMsg = 1;
				led = 7;
			}
			// IF 'o' OR 'O'
			else if (Rec_Reg == 79 || Rec_Reg == 111)
			{
				Flag_New_Data = 0;
				countMsg = 1;
				led = 8;
			}
			// IF 'g' OR 'G'
			else if (Rec_Reg == 71 || Rec_Reg == 103)
			{
				Flag_New_Data = 0;
				countMsg = 1;
				led = 9;
			}
			
			else
			{
				Trans_String(n1,msg1);
				Flag_New_Data = 0;
				countMsg = 0;
			}
		}
		
		// IF Condition for second [0:2] key selector
		else if (Flag_New_Data == 1 && countMsg == 1)
		{
			Flag_New_Data = 0;
			countMsg = 0;
			
			switch(led)
			{
				// For RED LED
				case (6):		
					if (Rec_Reg == 48)  // For '0' OFF
					{
						GPIOC->ODR &= ~(1 << 6);
						Trans_String(n3,msg3);
					}
					else if (Rec_Reg == 49)	// For '1' ON
					{
						GPIOC->ODR |= (1 << 6);
						Trans_String(n4,msg4);
						
					}
					else if (Rec_Reg == 50)	// For '2' TOGGLE
					{
						GPIOC->ODR ^= (1 << 6);
						Trans_String(n5,msg5);
					}
					else
					{
						Trans_String(n1,msg1);
					}
					break;
					
				// For BLUE LED
				case (7):		
					if (Rec_Reg == 48)  // For '0' OFF
					{
						GPIOC->ODR &= ~(1 << 7);
						Trans_String(n6,msg6);
					}
					else if (Rec_Reg == 49)	// For '1' ON
					{
						GPIOC->ODR |= (1 << 7);
						Trans_String(n7,msg7);
						
					}
					else if (Rec_Reg == 50)	// For '2' TOGGLE
					{
						GPIOC->ODR ^= (1 << 7);
						Trans_String(n8,msg8);
					}
					else
					{
						Trans_String(n1,msg1);
					}
					break;
				
				// For ORANGE LED
				case (8):		
					if (Rec_Reg == 48)  // For '0' OFF
					{
						GPIOC->ODR &= ~(1 << 8);
						Trans_String(n9,msg9);
					}
					else if (Rec_Reg == 49)	// For '1' ON
					{
						GPIOC->ODR |= (1 << 8);
						Trans_String(n10,msg10);
						
					}
					else if (Rec_Reg == 50)	// For '2' TOGGLE
					{
						GPIOC->ODR ^= (1 << 8);
						Trans_String(n11,msg11);
					}
					else
					{
						Trans_String(n1,msg1);
					}
					break;
					
				// For GREEN LED
				case (9):		
					if (Rec_Reg == 48)  // For '0' OFF
					{
						GPIOC->ODR &= ~(1 << 9);
						Trans_String(n12,msg12);
					}
					else if (Rec_Reg == 49)	// For '1' ON
					{
						GPIOC->ODR |= (1 << 9);
						Trans_String(n13,msg13);
						
					}
					else if (Rec_Reg == 50)	// For '2' TOGGLE
					{
						GPIOC->ODR ^= (1 << 9);
						Trans_String(n14,msg14);
					}
					else
					{
						Trans_String(n1,msg1);
					}
					break;
				
				default:
					Trans_String(n1,msg1);
					
			}
		}
		
  }

}


void USART3_4_IRQHandler(void)
{
	// Save the receive register’s value into a global variable
	Rec_Reg = USART3->RDR;
	// Set a global variable as a flag indicating new data
		Flag_New_Data = 1;
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
