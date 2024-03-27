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
	// ------5.2 — Setting the GPIO Modes------ //
	
	// RCC to enable the GPIOC and GPIOB peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
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
	// For SEL13 AF1 [0101] for bit [23:20]
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
	
	// ------5.3 — Initializing the I2C Peripheral------ //
	
	// Enable the I2C2 peripheral in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
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
	
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// For I2C2 -> SADD = 0x69 (1101001) for bit [7:1] 
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field
	// For I2C2 -> NBYTES = 2  for bit [23:16]
	I2C2->CR2 |= (2 << 16); 
	
	// Configure the RD_WRN to indicate a write operation
	// For I2C2 -> RD_WRN [0] for bit 10
	I2C2->CR2 &= ~(1 << 10);
	// Setting the START bit to begin the address frame
	// For I2C2 -> START [1] for bit 13
	I2C2->CR2 |= (1 << 13);
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	
	//														//
	//														//
	//  FOR ENABLING GYROSCOPE		//
	//														//
	//														//
	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set
	while(!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4)))
	{
		// Waiting...
	}
	
	// SAK
	if (I2C2->ISR & (1 << 4))
	{
		// ERROR...
	}		
	
	// TXIS Flag Set 
	if (I2C2->ISR & (1 << 1))
	{		
		// Write the address of the “CTRL_REG1” register into the I2C transmit register
		// For I2C2 -> TXDR = 0x20 (0-0100000) for bit [7:0]
		I2C2->TXDR |= (0x20 << 0);
	}	
	
	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set
	while(!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4)))
	{
		// Waiting...
	}
	
	// SAK
	if (I2C2->ISR & (1 << 4))
	{
		// ERROR...
	}	

	// TXIS Flag Set 
	if (I2C2->ISR & (1 << 1))
	{		
		// Write the instruction of the “CTRL_REG1” register into the I2C transmit register
		// For I2C2 -> TXDR = 00001011 for bit [7:0]
		I2C2->TXDR |= (1 << 0);
		I2C2->TXDR |= (1 << 1);
		I2C2->TXDR &= ~(1 << 2);
		I2C2->TXDR |= (1 << 3);
		I2C2->TXDR &= ~(1 << 4);
		I2C2->TXDR &= ~(1 << 5);
		I2C2->TXDR &= ~(1 << 6);
		I2C2->TXDR &= ~(1 << 7);
	}	
	
	// Wait until the TC (Transfer Complete) flag is set
	while(!(I2C2->ISR & (1 << 6)))
	{
		// Waiting...
	}
	
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// For I2C2 -> SADD = 0x69 for bit [7:1] 
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field
	// For I2C2 -> NBYTES = 1  for bit [23:16]
	I2C2->CR2 |= (1 << 16); 
	
	// Configure the RD_WRN to indicate a write operation
	// For I2C2 -> RD_WRN [0] for bit 10
	I2C2->CR2 &= ~(1 << 10);
	// Setting the START bit to begin the address frame
	// For I2C2 -> START [1] for bit 13
	I2C2->CR2 |= (1 << 13);
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	
	// 				LED SETUP 					//
	//														//
	//														//
	//														//
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
	
	GPIOC->ODR &= ~(1 << 6);
	GPIOC->ODR &= ~(1 << 7);
	GPIOC->ODR &= ~(1 << 8);
	GPIOC->ODR &= ~(1 << 9);
	//														//
	// 				LED SETUP 					//
	//														//
	//														//
	
	// Global Variables
	int16_t X_DATA;
	int16_t Y_DATA;
	int8_t X_L_DATA;
	int8_t X_H_DATA;
	int8_t Y_L_DATA;
	int8_t Y_H_DATA;
	
	
	
	
  while (1)
  {
		
	HAL_Delay(100); // Delay 100ms	
	//														//
	//														//
	//  			X_LOW - DATA				//
	//														//
	//														//
	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set
	while(!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4)))
	{
		// Waiting...
	}
	
	// SAK
	if (I2C2->ISR & (1 << 4))
	{
		// ERROR...
	}
			
	// TXIS Flag Set 
	if (I2C2->ISR & (1 << 1))
	{		
		// Write the address of the “OUT_X_L” register into the I2C transmit register
		// For I2C2 -> TXDR = 0x28 for bit [7:0]
		I2C2->TXDR |= (0x28 << 0);
	}	
	
	// Wait until the TC (Transfer Complete) flag is set
	while(!(I2C2->ISR & (1 << 6)))
	{
		// Waiting...
	}
	
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// For I2C2 -> SADD = 0x69 for bit [7:1] 
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field
	// For I2C2 -> NBYTES = 1 for bit [23:16]
	I2C2->CR2 |= (1 << 16); 
	// Configure the RD_WRN to indicate a read operation
	// For I2C2 -> RD_WRN [1] for bit 10
	I2C2->CR2 |= (1 << 10);
	// Setting the START bit to begin the address frame
	// For I2C2 -> START [1] for bit 13
	I2C2->CR2 |= (1 << 13);
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	//														//
	
	// Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not-Acknowledge) flags are set.
	while(!(I2C2->ISR & (1 << 2)) && !(I2C2->ISR & (1 << 4)))
	{
		// Waiting...
	}
	
	// SAK	
	if (I2C2->ISR & (1 << 4))
	{
		// ERROR...
	}
		
	if (I2C2->ISR & (1 << 2))
	{
		X_L_DATA = I2C2->RXDR;
	}
	
	// Wait until the TC (Transfer Complete) flag is set
	while(!(I2C2->ISR & (1 << 6)))
	{
		// Waiting...
	}
	//														//
	//  			X_LOW - DATA				//
	//														//
	//														//
	
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// For I2C2 -> SADD = 0x69 for bit [7:1] 
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field
	// For I2C2 -> NBYTES = 1 for bit [23:16]
	I2C2->CR2 |= (1 << 16);
	// Configure the RD_WRN to indicate a write operation
	// For I2C2 -> RD_WRN [0] for bit 10
	I2C2->CR2 &= ~(1 << 10);
	// Setting the START bit to begin the address frame
	// For I2C2 -> START [1] for bit 13
	I2C2->CR2 |= (1 << 13);
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	//														//
	
	//														//
	//														//
	//  			X_HIGH - DATA				//
	//														//
	//														//
	// Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set
	while(!(I2C2->ISR & (1 << 1)) && !(I2C2->ISR & (1 << 4)))
	{
		// Waiting...
	}
	
	// SAK
	if (I2C2->ISR & (1 << 4))
	{
		// ERROR...
	}
			
	// TXIS Flag Set 
	if (I2C2->ISR & (1 << 1))
	{		
		// Write the address of the “OUT_X_H” register into the I2C transmit register
		// For I2C2 -> TXDR = 0x29 for bit [7:0]
		I2C2->TXDR |= (0x29 << 0);
	}	
	
	// Wait until the TC (Transfer Complete) flag is set
	while(!(I2C2->ISR & (1 << 6)))
	{
		// Waiting...
	}
	
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// For I2C2 -> SADD = 0x69 for bit [7:1] 
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field
	// For I2C2 -> NBYTES = 1 for bit [23:16]
	I2C2->CR2 |= (1 << 16); 
	// Configure the RD_WRN to indicate a read operation
	// For I2C2 -> RD_WRN [1] for bit 10
	I2C2->CR2 |= (1 << 10);
	// Setting the START bit to begin the address frame
	// For I2C2 -> START [1] for bit 13
	I2C2->CR2 |= (1 << 13);
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	//														//
	
	// Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not-Acknowledge) flags are set.
	while(!(I2C2->ISR & (1 << 2)) && !(I2C2->ISR & (1 << 4)))
	{
		// Waiting...
	}
	
	// SAK	
	if (I2C2->ISR & (1 << 4))
	{
		// ERROR...
	}
		
	if (I2C2->ISR & (1 << 2))
	{
		X_H_DATA = I2C2->RXDR;
	}
	
	// Wait until the TC (Transfer Complete) flag is set
	while(!(I2C2->ISR & (1 << 6)))
	{
		// Waiting...
	}
	//														//
	//  			X_LOW - DATA				//
	//														//
	//														//
	
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	// Set the transaction parameters in the CR2 register
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// For I2C2 -> SADD = 0x69 for bit [7:1] 
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of data byte to be transmitted in the NBYTES[7:0] bit field
	// For I2C2 -> NBYTES = 1 for bit [23:16]
	I2C2->CR2 |= (1 << 16); 
	// Configure the RD_WRN to indicate a write operation
	// For I2C2 -> RD_WRN [0] for bit 10
	I2C2->CR2 &= ~(1 << 10);
	// Setting the START bit to begin the address frame
	// For I2C2 -> START [1] for bit 13
	I2C2->CR2 |= (1 << 13);
	//														//
	//														//
	//  		START/RESTART					//
	//														//
	//														//
	//														//
	
	// Concadenate DATA
  X_DATA = (X_L_DATA << 8) | X_H_DATA;
	
	// LED Controller with a 5 dps tolerance
	// Y: (+)RED(PC6) 		(-)BLUE(PC7) 
	// X: (+)ORANGE(PC8) 	(-)GREEN(PC9)
	
	
	if (X_DATA > 0)
	{
		GPIOC->ODR &= ~(1 << 6);
		GPIOC->ODR &= ~(1 << 7);
		GPIOC->ODR |= (1 << 8);
		GPIOC->ODR &= ~(1 << 9);
	}
	
	if (X_DATA < 0)
	{
		GPIOC->ODR &= ~(1 << 6);
		GPIOC->ODR &= ~(1 << 7);
		GPIOC->ODR &= ~(1 << 8);
		GPIOC->ODR |= (1 << 9);
	}
	
	HAL_Delay(100); // Delay 100ms	
	
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
