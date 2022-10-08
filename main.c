/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct structSevenSegCnfg {
	GPIO_TypeDef* gpio;
	uint16_t pin;
} sevenSegCnfg;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LO_FREQ_BUZZ 764
#define HI_FREQ_BUZZ 270
#define MAX_INTRVL_LO_FREQ 200
#define MAX_INTRVL_HI_FREQ 400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t MSG[35] = {'\0'};
// counts TIM2 interrupts to end up with a 1 s interval
// after each interval a message, with the uValDisp value,
// is send over the serial interface
static uint8_t cPeriod = 0;
static uint8_t uValDisp = 0;
static uint8_t uNibble1 = 0;
static uint8_t uNibble2 = 0;
const sevenSegCnfg SEVEN_SEG_CNFG[7] = {
		{GPIOA, SEG_A_Pin}
		, {GPIOA, SEG_B_Pin}
		, {GPIOC, SEG_C_Pin}
		, {GPIOC, SEG_D_Pin}
		, {GPIOA, SEG_E_Pin}
		, {GPIOA, SEG_F_Pin}
		, {GPIOA, SEG_G_Pin}
};
const uint8_t SEVEN_SEG_DGT[16] = {
		0xC0	//192 ~063 0
		, 0xF9	//249 ~006 1
		, 0xA4	//164 ~091 2
		, 0xB0	//176 ~079 3
		, 0x99	//153 ~102 4
		, 0x92	//146 ~109 5
		, 0x82	//130 ~125 6
		, 0xF8  //248 ~007 7
		, 0x80  //128 ~127 8
		, 0x90  //144 ~111 9
		, 0x88	//136 ~119 A
		, 0x83	//131 ~124 B
		, 0xC6	//198 ~057 C
		, 0xA1	//161 ~094 D
		, 0x86	//134 ~121 E
		, 0x8E	//142 ~113 F
		// dp and uc/lc are not used so far
		//, 0x7F	//127 ~128 dp
		//, 0xFF	//255 ~000 uc/lc
};
static bool bAlternateBuzzPin = true;
static uint16_t iFreqBuzz = 0;
static uint16_t cIntervalBuzz = 0;
static uint16_t uMaxIntrvlBuzz = 0;
static bool bSetToZero = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool SEVEN_SEG_WriteDigit(const uint8_t);

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
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	// A) start timer TIM2
	HAL_TIM_Base_Start_IT(&htim2);
	// B) set display to indicate '00'
	SEVEN_SEG_WriteDigit(0);
	// C) provide Vdd for both digits
	// set common anode segment 1 high
	HAL_GPIO_WritePin(GPIOA, CA1_Pin, GPIO_PIN_SET);
	// set common anode segment 2 high
	HAL_GPIO_WritePin(GPIOA, CA2_Pin, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// multiplex seven segment digits
		// A) shut off both seven segment displays
		HAL_GPIO_WritePin(GPIOA, CA1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CA2_Pin, GPIO_PIN_RESET);
		// B1) write to seven segment digit 1 and turn on momentarily
		HAL_GPIO_WritePin(GPIOA, CA1_Pin, GPIO_PIN_SET);
		SEVEN_SEG_WriteDigit(uNibble1);
		HAL_Delay(1); // there has to be a slight delay
		HAL_GPIO_WritePin(GPIOA, CA1_Pin, GPIO_PIN_RESET);
		// B2) write to seven segment digit 2 and turn on momentarily
		HAL_GPIO_WritePin(GPIOA, CA2_Pin, GPIO_PIN_SET);
		SEVEN_SEG_WriteDigit(uNibble2);
		HAL_Delay(1); // there has to be a slight delay
		HAL_GPIO_WritePin(GPIOA, CA2_Pin, GPIO_PIN_RESET);
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

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//****************************************************************************
//*                     SEVEN_SEG_WriteDigit
//****************************************************************************
bool SEVEN_SEG_WriteDigit(const uint8_t uNibble)
{
	uint8_t bit_mask = 1;
	for (int i = 0; i < 7; i++)
	{
		HAL_GPIO_WritePin(SEVEN_SEG_CNFG[i].gpio
				, SEVEN_SEG_CNFG[i].pin
				, SEVEN_SEG_DGT[uNibble] & bit_mask);
		bit_mask = bit_mask << 1;
	}
	return true;
}
//****************************************************************************
//*                     HAL_TIM_PeriodElapsedCallback
//****************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// handle callback from TIM2
	if (htim->Instance == htim2.Instance)
	{
		// A)
		// test button 1
		if (!HAL_GPIO_ReadPin(GPIOA, BTN1_Pin))
		{
			// check if uValDisp is at zero
			if (uValDisp == 0)
			{
				HAL_TIM_Base_Stop_IT(&htim3);
				cIntervalBuzz = 0;
				uMaxIntrvlBuzz = MAX_INTRVL_LO_FREQ;
				// configure TIM3
				htim3.Instance->ARR = iFreqBuzz = LO_FREQ_BUZZ;
				// start TIM3 to generate frequency for buzzer
				HAL_TIM_Base_Start_IT(&htim3);
			}
			else
			{
				--uValDisp;
			}
		}
		// test button 2
		if (!HAL_GPIO_ReadPin(GPIOA, BTN2_Pin))
		{
			// check if uValDisp is at full range
			if (uValDisp == 0xFF)
			{
				HAL_TIM_Base_Stop_IT(&htim3);
				cIntervalBuzz = 0;
				uMaxIntrvlBuzz = MAX_INTRVL_HI_FREQ;
				// configure TIM3
				htim3.Instance->ARR = iFreqBuzz = HI_FREQ_BUZZ;
				// start TIM3 to generate frequency for buzzer
				HAL_TIM_Base_Start_IT(&htim3);
			}
			else
			{
				++uValDisp;
			}
		}
		// test button 3
		if (!HAL_GPIO_ReadPin(GPIOA, BTN3_Pin))
		{
			if (bSetToZero)
			{
				uValDisp = 0;
			}
			else
			{
				uValDisp = 0xFF;
			}
			bSetToZero = !bSetToZero;
		}
		// B) split into nibble
		uNibble1 = (uValDisp >> 4) & 0x0F;
		uNibble2 = uValDisp & 0x0F;
		// C) bring the uValDisp value to the outside world
		if (cPeriod == 5)
		{
			// send message over the serial interface
			sprintf((char*)MSG, "uValDisp is %d\r\n", uValDisp);
			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
			cPeriod = 0;
		}
		else
		{
			++cPeriod;
		}
	}
	// handle callback from TIM3
	if (htim->Instance == htim3.Instance)
	{
		if ((++cIntervalBuzz) == uMaxIntrvlBuzz)
		{
			HAL_TIM_Base_Stop_IT(&htim3);
			return;
		}
		if (bAlternateBuzzPin)
		{
			HAL_GPIO_WritePin(GPIOC, BUZZ1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, BUZZ2_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, BUZZ1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, BUZZ2_Pin, GPIO_PIN_SET);
		}
		bAlternateBuzzPin = !bAlternateBuzzPin;
	}
}
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
