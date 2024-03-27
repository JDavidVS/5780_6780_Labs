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

#include "main.h"


#define I3G4250D 							(0x69)
#define CTRL_REG1							(0x20)
#define CTRL_REG1_XY_DATA			(0x0B)
#define OUT_X									(0xA8)
#define OUT_Y									(0xAA)

void SystemClock_Config(void);

void writeSingleByte_I2C(char SAD, char SUB);
void writeTwoByte_I2C(char SAD, char SUB, char DATA);
int readByte_I2C(char SAD, int NBYTES);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  SystemClock_Config();
	
	// RCC to enable the GPIOC, GPIOB and I2C peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function
	// For PB11 -> MODER11 [10] for bit 23 and 22
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER &= ~(1 << 22);
	// For PB11 -> OTYPER11 [1] for bit 11
	GPIOB->OTYPER |= (1 << 11);
	// PB11 -> I2C2_SDA -> AF1
	// For SEL11 AF1 [0001] for bit [15:12]
	GPIOB->AFR[1] |= (1 << 12);
	GPIOB->AFR[1] &= ~((1 << 13) | (1 << 14) | (1 << 15));
	
	// Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function
	// For PB13 -> MODER13 [10] for bit 27 and 26
	GPIOB->MODER |= (1 << 27);
	GPIOB->MODER &= ~(1 << 26);
	// For PB13 -> OTYPER13 [1] for bit 13
	GPIOB->OTYPER |= (1 << 13);
	// PB13 -> I2C2_SCL -> AF5
	// For SEL13 AF5 [0101] for bit [23:20]
	GPIOB->AFR[1] &= ~(1 << 23);
	GPIOB->AFR[1] |= (1 << 22);
	GPIOB->AFR[1] &= ~(1 << 21);
	GPIOB->AFR[1] |= (1 << 20);
	
	// Set PB14 to output mode, push-pull output type, and initialize/set the pin high
	// For PB14 -> MODER14 [01] for bit 29 and 28
	GPIOB->MODER &= ~(1 << 29);
	GPIOB->MODER |= (1 << 28);
	// For PB14 -> OTYPER14 [0] for bit 14
	GPIOB->OTYPER &= ~(1 << 14);
	// FOR PB14 -> ODR14 [1] for bit 14
	GPIOB->ODR |= (1 << 14); 
	
	// Set PC0 to output mode, push-pull output type, and initialize/set the pin high
	// For PC0 -> MODER0 [01] for bit 1 and 0
	GPIOC->MODER &= ~(1 << 1);
	GPIOC->MODER |= (1 << 0);
	// For PC0 -> OTYPER0 [0] for bit 0
	GPIOC->OTYPER &= ~(1 << 0);
	// FOR PC0 -> ODR0 [1] for bit 0
	GPIOC->ODR |= (1 << 0);
	
	// Set PB15 to input mode
	// For PB15 -> MODER15 [00] for bit 31 and 30
	GPIOB->MODER &= ~(1 << 31);
	GPIOB->MODER &= ~(1 << 30);
	
	// Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C
	// From Table 91 of Peripheral Manual
	// For I2C2 -> PRESC = 1 for bit [31:28]
	I2C2->TIMINGR &= ~(1 << 31);
	I2C2->TIMINGR &= ~(1 << 30);
	I2C2->TIMINGR &= ~(1 << 29);
	I2C2->TIMINGR |= (1 << 28);
	// For I2C2 -> SCLL = 0x13 (00010011) for bit [7:0]
	I2C2->TIMINGR |= (1 << 0);
	I2C2->TIMINGR |= (1 << 1);
	I2C2->TIMINGR &= ~(1 << 2);
	I2C2->TIMINGR &= ~(1 << 3);
	I2C2->TIMINGR |= (1 << 4);
	I2C2->TIMINGR &= ~(1 << 5);
	I2C2->TIMINGR &= ~(1 << 6);
	I2C2->TIMINGR &= ~(1 << 7);
	// For I2C2 -> SCLH = 0xF (00001111) for bit [15:8]
	I2C2->TIMINGR |= (1 << 8);
	I2C2->TIMINGR |= (1 << 9);
	I2C2->TIMINGR |= (1 << 10);
	I2C2->TIMINGR |= (1 << 11);
	I2C2->TIMINGR &= ~(1 << 12);
	I2C2->TIMINGR &= ~(1 << 13);
	I2C2->TIMINGR &= ~(1 << 14);
	I2C2->TIMINGR &= ~(1 << 15);
	// For I2C2 -> SDADEL = 0x2 (0010) for bit [19:16]
	I2C2->TIMINGR &= ~(1 << 16);
	I2C2->TIMINGR |= (1 << 17);
	I2C2->TIMINGR &= ~(1 << 18);
	I2C2->TIMINGR &= ~(1 << 19);
	// For I2C2 -> SCLDEL =  0x4 (0100) for bit [23:20]
	I2C2->TIMINGR &= ~(1 << 20);
	I2C2->TIMINGR &= ~(1 << 21);
	I2C2->TIMINGR |= (1 << 22);
	I2C2->TIMINGR &= ~(1 << 23);
	
	// Enable the I2C peripheral using the PE bit in the CR1 register
	I2C2->CR1 |= (1 << 0);
	
	
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
	// Set LEDs to LOW
	GPIOC->ODR &= ~(1 << 6);
	GPIOC->ODR &= ~(1 << 7);
	GPIOC->ODR &= ~(1 << 8);
	GPIOC->ODR &= ~(1 << 9);
	//														//
	// 				LED SETUP 					//
	
	//														//
	// 				GYROSCOPE						//
	//														//
	// Enable the X and Y sensing axes in the CTRL_REG1 register
	writeTwoByte_I2C(I3G4250D, CTRL_REG1, CTRL_REG1_XY_DATA);
	// Setting the STOP bit
	I2C2->CR2 |= I2C_CR2_STOP; 
	// Wait until stop flag is set
	while (I2C2->CR2 & I2C_CR2_STOP)
	{
		//Waiting...
	}

	// Variables
	int16_t X_DATA = 0;
	int16_t Y_DATA = 0;
	
  while (1)
  {
		//														//
		// 				X-DATA							//
		//														//
		// Reading register for X-value
		writeSingleByte_I2C(I3G4250D, OUT_X);
		X_DATA = readByte_I2C(I3G4250D, 2);
		// Setting the STOP bit
		I2C2->CR2 |= I2C_CR2_STOP; 
		// Wait until stop flag is set
		while (I2C2->CR2 & I2C_CR2_STOP)
		{
			//Waiting...
		}
		
		//														//
		// 				Y-DATA							//
		//														//
		// Reading register for Y-value
		writeSingleByte_I2C(I3G4250D, OUT_Y);
		Y_DATA = readByte_I2C(I3G4250D, 2);
		// Setting the STOP bit
		I2C2->CR2 |= I2C_CR2_STOP; 
		// Wait until stop flag is set
		while (I2C2->CR2 & I2C_CR2_STOP)
		{
			//Waiting...
		}
	
	
		// LED Controller
		// Y: (+)RED(PC6) 		(-)BLUE(PC7) 
		// X: (+)ORANGE(PC8) 	(-)GREEN(PC9)
	
		if (X_DATA > 2000)
		{
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR |= (1 << 9);
		}
		
		if (X_DATA < -2000)
		{
			GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
		
		if (Y_DATA > 2000)
		{
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 7);
		}
		
		else if (Y_DATA < -2000)
		{
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR |= (1 << 7);
		}
			
		HAL_Delay(100); // Delay 100ms	
	
  }
  /* USER CODE END 3 */
}

/**
  * @brief Write one byte to I2C
  * @retval None
  */
void writeSingleByte_I2C(char SAD, char SUB)
{		
		// Clear SADD and NBYTES field
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		// Set SADD address
		I2C2->CR2 |= (SAD << 1);
	  // Set the number of data byte to be transmitted 
		I2C2->CR2 |= (1 << 16);
		// Configure the RD_WRN to indicate a WRITE operation
		I2C2->CR2 &= ~(1 << 10);
	  // Software end mode
		I2C2->CR2 &= ~(I2C_CR2_AUTOEND); 
		// Setting the START bit to begin the address frame
		I2C2->CR2 |= (1 << 13);
	
		while ((I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0)
		{
			// Waiting...
			HAL_Delay(1);
		}
		
		// Write the address of the register into the I2C transmit register
	  I2C2->TXDR = SUB;
		
		// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set
		while ((I2C2->ISR & I2C_ISR_TC) == 0)
		{
			// Waiting...
		}
		return;
}

/**
  * @brief Write two bytes to I2C
  * @retval None
  */
void writeTwoByte_I2C(char SAD, char SUB, char DATA)
{
		// Clear SADD and NBYTES field
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		// Set SADD address
		I2C2->CR2 |= (SAD << 1);
	  // Set the number of data byte to be transmitted 
		I2C2->CR2 |= (2 << 16);
		// Configure the RD_WRN to indicate a WRITE operation
		I2C2->CR2 &= ~(1 << 10);
		// Software end mode
		I2C2->CR2 &= ~(I2C_CR2_AUTOEND); 
		// Setting the START bit to begin the address frame
		I2C2->CR2 |= (1 << 13);
	
		while ((I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0)
		{
			// Waiting...
			HAL_Delay(1);
		}
		
		// Write the address of the register into the I2C transmit register
	  I2C2->TXDR = SUB;
		
		while ((I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) == 0)
		{
			// Waiting...
		}
		
		// Write the address of the register into the I2C transmit register
	  I2C2->TXDR = DATA;
		
		// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set
		while ((I2C2->ISR & I2C_ISR_TC) == 0)
		{
			// Waiting...
		}
		return;
}

/**
  * @brief Read N-bytes to I2C
  * @retval Int
  */
int readByte_I2C(char SAD, int NBYTES)
{		
		// Local variables
		int DATA = 0;
	
		// Clear SADD and NBYTES field
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		// Set SADD address
		I2C2->CR2 |= (SAD << 1);
	  // Set the number of data byte to be transmitted 
		I2C2->CR2 |= (NBYTES << 16);
		// Configure the RD_WRN to indicate a READ operation
		I2C2->CR2 |= (1 << 10);
		// Software end mode
		I2C2->CR2 &= ~(I2C_CR2_AUTOEND); 
		// Setting the START bit to begin the address frame
		I2C2->CR2 |= (1 << 13);
	
		for (int k =0; k < NBYTES; k++)
		{
			// Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not-Acknowledge) flags are set.
			while ((I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)) == 0)
			{
				// Waiting...
				HAL_Delay(1);
			}
			
			DATA |= I2C2->RXDR << k*8;
		}
		
		while ((I2C2->ISR & I2C_ISR_TC) == 0)
		{
			// Waiting...
		}
		return DATA;
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
