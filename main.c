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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct tagTIMING_EDGE_IR
{
	uint8_t edge;
	uint32_t uFallingEdge;
	uint32_t uRisingEdge;
	int32_t bitTiming;
} TIMING_EDGE_IR, *PTIMING_EDGE_IR;

#define LEN_MAX_ENTRY		31
typedef struct tagFRAME_TRANSMIT
{
    const char soh;
    uint16_t cmnd;
    const char stx;
    char payload[LEN_MAX_ENTRY];
    const char etx;
    const char etb;
    const char eot;
} FRAME_TRANSMIT, *PFRAME_TRANSMIT;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENTRY_MAX			26
#define NOF_EDGE			74
#define DURATION_PULSE  	450 // ms

#define SOH					1
#define STX					2
#define ETX					3
#define ETB					23
#define EOT					4
#define LEN_FRAME_TRANSMIT	38
#define LEN_CRC				4

#define DELAY_4HZ_SERIAL	125

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// the ir-remote sends a frame packed in 64 bit
__IO static uint64_t frameIr = 0;
__IO static uint8_t cEdge = 0;
__IO static TIMING_EDGE_IR timingEdgeIr[NOF_EDGE] = { 0 };
char aKey[ENTRY_MAX][LEN_MAX_ENTRY] = {0};
uint64_t hexCodeIr = 0;
uint8_t indexKey = 0;

// this is the way to initialise the attributes, which are constant,
// in the struct, and also on the fly
// Initialise the attributes, which are not constant, in the struct
FRAME_TRANSMIT frameTransmit = { SOH, 0, STX, { '\0' }, ETX, ETB, EOT };
uint32_t valCrc = 0;

uint8_t chBuffer[LEN_FRAME_TRANSMIT + LEN_CRC] = { '\0' };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void initKey();
static void decode();
static void evaluate();
static void writeBit(const uint8_t bit);
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
  // set the definitions for the keys on the remote
  // into an array
  initKey();

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  // start both channels on TIM2
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (cEdge == NOF_EDGE)
	{
	  // stop both channels on TIM2
	  HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	  // essential to get it working!!!
	  // (concurrency or, it might be that UART2 is using TIM2...)
	  HAL_Delay(DELAY_4HZ_SERIAL);
	  evaluate();
	  // default state
	  cEdge = 0;
	  frameIr = 0;
	  // start both channels on TIM2
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1890 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//////////////////////////////////////////////////////////////////////////////
//****************************************************************************
//*                     HAL_TIM_IC_CaptureCallback
//****************************************************************************
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	// channel_1 interrupt, falling edge
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		// just checking
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

		timingEdgeIr[cEdge].edge = 0;
		timingEdgeIr[cEdge].uFallingEdge =
				HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		timingEdgeIr[cEdge].uRisingEdge = 0;
		++cEdge;
	}
	// channel_2 interrupt, rising edge
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		// just checking
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

		timingEdgeIr[cEdge].edge = 1;
		timingEdgeIr[cEdge].uFallingEdge = 0;
				HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		timingEdgeIr[cEdge].uRisingEdge =
				HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		++cEdge;
	}
}
//****************************************************************************
//*                     initKey
//****************************************************************************
void initKey()
{
	sprintf(aKey[0], "%s", "radio                         ");
	sprintf(aKey[1], "%s", "menu | text                   ");
	sprintf(aKey[2], "%s", "forward                       ");
	sprintf(aKey[3], "%s", "stop                          ");
	sprintf(aKey[4], "%s", "record                        ");
	sprintf(aKey[5], "%s", "rewind                        ");
	sprintf(aKey[6], "%s", "blue                          ");
	sprintf(aKey[7], "%s", "guide | tv home               ");
	sprintf(aKey[8], "%s", "louder                        ");
	sprintf(aKey[9], "%s", "quieter                       ");
	sprintf(aKey[10], "%s", "pause/play | back/?           ");
	sprintf(aKey[11], "%s", "P+ | red | av                 ");
	sprintf(aKey[12], "%s", "P- | green | right | audio off");
	sprintf(aKey[13], "%s", "yellow                        ");
	sprintf(aKey[14], "%s", "0 | ' '                       ");
	sprintf(aKey[15], "%s", "i-tv                          ");
	sprintf(aKey[16], "%s", "9 | wxyz | down               ");
	sprintf(aKey[17], "%s", "8 | tuv | left | up           ");
	sprintf(aKey[18], "%s", "OK                            ");
	sprintf(aKey[19], "%s", "5 | jkl                       ");
	sprintf(aKey[20], "%s", "4 | 6 | ghi | mno             ");
	sprintf(aKey[21], "%s", "7 | pqrs                      ");
	sprintf(aKey[22], "%s", "2 | abc                       ");
	sprintf(aKey[23], "%s", "3 | def                       ");
	sprintf(aKey[24], "%s", "1 | ;@?!                      ");
	sprintf(aKey[25], "%s", "INVALID                       ");
}
//****************************************************************************
//*                     decode
//****************************************************************************
static void decode()
{
	hexCodeIr = frameIr & 0xFFF;
	switch (hexCodeIr)
	{
	case 0xABF: // radio
		indexKey = 0;
		break;
	case 0xAEF: // menu | text
		indexKey = 1;
		break;
	case 0xBBB: // forward
		indexKey = 2;
		break;
	case 0xBEB: // stop
		indexKey = 3;
		break;
	case 0xBEF: // record
		indexKey = 4;
		break;
	case 0xBFB: // rewind
		indexKey = 5;
		break;
	case 0xBFE: // blue
		indexKey = 6;
		break;
	case 0xEBB: // guide | tv home
		indexKey = 7;
		break;
	case 0xEBF: // louder
		indexKey = 8;
		break;
	case 0xEEB: // quieter
		indexKey = 9;
		break;
	case 0xEEF: // pause/play | back/?
		indexKey = 10;
		break;
	case 0xEFB: // P+ | red | av
		indexKey = 11;
		break;
	case 0xEFE: // P- | green | right | audio off
		indexKey = 12;
		break;
	case 0xEFF: // yellow
		indexKey = 13;
		break;
	case 0xFAB: // 0
		indexKey = 14;
		break;
	case 0xFAF: // i-tv
		indexKey = 15;
		break;
	case 0xFBB: // 9 | down
		indexKey = 16;
		break;
	case 0xFBE: // 8 | left | up
		indexKey = 17;
		break;
	case 0xFBF: // OK
		indexKey = 18;
		break;
	case 0xFEB: // 5
		indexKey = 19;
		break;
	case 0xFEE: // 4 | 6
		indexKey = 20;
		break;
	case 0xFEF: // 7
		indexKey = 21;
		break;
	case 0xFFA: // 2
		indexKey = 22;
		break;
	case 0xFFB: // 3
		indexKey = 23;
		break;
	case 0xFFE: // 1
		indexKey = 24;
		break;
	default: // INVALID
		indexKey = 25;
		break;
	} // eof switch

	frameTransmit.cmnd = hexCodeIr;
	sprintf(frameTransmit.payload
			, "%s"
			, aKey[indexKey]
	);

	chBuffer[0] = frameTransmit.soh;
	chBuffer[1] = 0;
	chBuffer[1] = (frameTransmit.cmnd & 0x0F00) >> 8;
	chBuffer[2] = (frameTransmit.cmnd & 0x00FF);
	chBuffer[3] = frameTransmit.stx;
	for (uint8_t i = 0; i < LEN_MAX_ENTRY; i++)
	{
		chBuffer[i + 4] = frameTransmit.payload[i];
	}
	chBuffer[34] = '\0';
	chBuffer[35] = frameTransmit.etx;
	chBuffer[36] = frameTransmit.etb;
	chBuffer[37] = frameTransmit.eot;
	// calculate crc and feed into chBuffer
	valCrc = HAL_CRC_Calculate(&hcrc
			, (uint32_t*)chBuffer
			, LEN_FRAME_TRANSMIT
	);
	chBuffer[38] = (valCrc & 0xFF000000) >> 24;
	chBuffer[39] = (valCrc & 0x00FF0000) >> 16;
	chBuffer[40] = (valCrc & 0x0000FF00) >> 8;
	chBuffer[41] = (valCrc & 0x000000FF);
	// transmit over serial
	HAL_UART_Transmit(&huart2
			, (uint8_t*)chBuffer
			, LEN_FRAME_TRANSMIT + LEN_CRC
			, 100
	);

	// essential to get it working!!!
	// (concurrency or, it might be that UART2 is using TIM2...)
	HAL_Delay(DELAY_4HZ_SERIAL);
}
//****************************************************************************
//*                     evaluate
//****************************************************************************
static void evaluate()
{
	bool bEdge = false;
	uint8_t lastBit = 0;
	for (uint8_t i = 0; i < 36; i++)
	{
		if ((bool)timingEdgeIr[i].edge != bEdge)
		{
			// first edge is not a falling edge
			return;
		}
		if (i >= 1)
		{
			lastBit = frameIr & 1;
			if (i % 2 == 0)
			{
				// falling edge
				timingEdgeIr[i].bitTiming =
						timingEdgeIr[i - 1].uRisingEdge -
						timingEdgeIr[i].uFallingEdge;
				if (lastBit == 0
						&& (timingEdgeIr[i].uFallingEdge > 0
						&& timingEdgeIr[i].uFallingEdge < DURATION_PULSE))
				{
					writeBit(0);
				}
				else
				{
					writeBit(1);
				}
			}
			else
			{
				// rising edge
				timingEdgeIr[i].bitTiming =
						timingEdgeIr[i - 1].uFallingEdge -
						timingEdgeIr[i].uRisingEdge;
				if (lastBit == 1
						&& (timingEdgeIr[i].uRisingEdge > 0
						&& timingEdgeIr[i].uRisingEdge < DURATION_PULSE))
				{
					writeBit(1);
				}
				else
				{
					writeBit(0);
				}
			}
		}
		bEdge = !bEdge;
	}
	decode();
}
//****************************************************************************
//*                     writeBit
//****************************************************************************
static void writeBit(const uint8_t bit)
{
	frameIr <<= 1;
	frameIr |= bit;
}
//////////////////////////////////////////////////////////////////////////////
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
