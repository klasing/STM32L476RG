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
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct tagDATA_PACKET
{
	uint8_t edge;
	uint32_t uFallingEdge;
	uint32_t uRisingEdge;
	int32_t bitTiming;
} DATA_PACKET;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_MAX	128
#define NOF_EDGE	74

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
__IO static uint8_t cEdge = 0;
__IO static DATA_PACKET data_packet[NOF_EDGE] = {0};
uint8_t MSG[BUFFER_MAX] = {'\0'};
uint8_t cFrameReceived = 0;
__IO static uint64_t frame = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void clear_buffer();
static void evaluate();
static void writeBit(const uint8_t bit);
static void decode(const uint64_t frame);
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
		  // clear message buffer
		  clear_buffer();
		  sprintf((char*)MSG, "%3d Frame received\r\n", ++cFrameReceived);
		  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		  // essential to get it working!!!
		  // (concurrency or, it might be that UART2 is using TIM2...)
		  HAL_Delay(50);
		  evaluate();
		  // default state
		  cEdge = 0;
		  frame = 0;
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
  htim2.Init.Period = 0xFFFFFFFF - 1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;//DISABLE;
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
		data_packet[cEdge].edge = 0;
		data_packet[cEdge].uFallingEdge =
				HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		data_packet[cEdge].uRisingEdge = 0;
		++cEdge;
	}
	// channel_2 interrupt, rising edge
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		// just checking
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		data_packet[cEdge].edge = 1;
		data_packet[cEdge].uFallingEdge = 0;
				HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		data_packet[cEdge].uRisingEdge =
				HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		++cEdge;
	}
}
//****************************************************************************
//*                     clear_buffer
//****************************************************************************
static void clear_buffer()
{
	for (uint8_t i = 0; i < BUFFER_MAX; i++) MSG[i] = '\0';
}
//****************************************************************************
//*                     evaluate
//****************************************************************************
static void evaluate()
{
	char sEdgeDirectionMessage[2][8] = {{'F','A','L','L','I','N','G','\0'}
			, {'R','I','S','I','N','G','\0'}
	};
	uint8_t cError = 0;
	uint8_t cEdge = 0;
	bool bEdge = false;
	uint8_t last_bit = 0;
	for (uint8_t i = 0; i < 36; i++)
	{
		// clear message buffer
		clear_buffer();
		if ((bool)data_packet[i].edge != bEdge)
		{
			sprintf((char*)MSG, "Error %3d: edge %2d is NOT %s\r\n", ++cError
					, cEdge++
					, sEdgeDirectionMessage[i % 2]);
			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
			return;
		}
		if (i >= 1)
		{
			last_bit = frame & 1;
			if (i % 2 == 0)
			{
				data_packet[i].bitTiming = data_packet[i - 1].uRisingEdge -
						data_packet[i].uFallingEdge;
				if (last_bit == 0 && (data_packet[i].uFallingEdge > 0
						&& data_packet[i].uFallingEdge < 450))
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
				data_packet[i].bitTiming = data_packet[i - 1].uFallingEdge -
						data_packet[i].uRisingEdge;
				if (last_bit == 1 && (data_packet[i].uRisingEdge > 0
						&& data_packet[i].uRisingEdge < 450))
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
	decode(frame);
}
//****************************************************************************
//*                     writeBit
//****************************************************************************
static void writeBit(const uint8_t bit)
{
	frame <<= 1;
	frame |= bit;
}
//****************************************************************************
//*                     decode
//****************************************************************************
//		, {'','','','','','','','','','','','','',''}
const char aKey[38][14] = {
		{'r','a','d','i','o','\0'}							// 0xABF
		, {'m','e','n','u','\0'}							// 0xAEF
		, {'t','e','x','t','\0'}							// 0xAEF
		, {'f','o','r','w','a','r','d','\0'} 				// 0xBBB
		, {'s','t','o','p','\0'} 							// 0xBEB
		, {'r','e','c','o','r','d','\0'} 					// 0xBEF
		, {'r','e','w','i','n','d','\0'} 					// 0xBFB
		, {'b','l','u','e','\0'} 							// 0xBFE
		, {'g','u','i','d','e','\0'} 						// 0xEBB
		, {'t','v',' ','h','o','m','e','\0'} 				// 0xEBB
		, {'l','o','u','d','e','r','\0'}					// 0xEBF
		, {'q','u','i','e','t','e','r','\0'}				// 0xEEB
		, {'p','a','u','s','e','/','p','l','a','y','\0'}	// 0xEEF
		, {'b','a','c','k','/','?','\0'} 					// 0xEEF
		, {'P','+','\0'}									// 0xEFB
		, {'r','e','d','\0'}								// 0xEFB
		, {'a','v','\0'}									// 0xEFB
		, {'P','-','\0'}									// 0xEFE
		, {'g','r','e','e','n','\0'}						// 0xEFE
		, {'r','i','g','h','t','\0'}						// 0xEFE
		, {'a','u','d','i','o',' ','o','f','f','\0'}		// 0xEFE
		, {'y','e','l','l','o','w','\0'}					// 0xEFF
		, {'0','\0'}										// 0xFAB
		, {'i','-','t','v','\0'}							// 0xFAF
		, {'d','o','w','n','\0'}							// 0xFBB
		, {'9','\0'}										// 0xFBB
		, {'l','e','f','t','\0'}							// 0xFBE
		, {'u','p','\0'}									// 0xFBE
		, {'8','\0'}										// 0XFBE
		, {'O','K','\0'}									// 0xFBF
		, {'5','\0'}										// 0XFEB
		, {'4','\0'}										// 0XFEE
		, {'6','\0'}										// 0XFEE
		, {'7','\0'}										// 0XFEF
		, {'2','\0'}										// 0XFFA
		, {'3','\0'}										// 0XFFB
		, {'1','\0'}										// 0XFFE
		, {'I','N','V','A','L','I','D','\0'}
};
static void decode(const uint64_t frame)
{
	uint8_t i = 0;
	switch (frame & 0xFFF)
	{
	case 0xABF: // radio
		i = 0;
		break;
	case 0xAEF: // menu | text
		i = 1;
		//i = 2;
		break;
	case 0xBBB: // forward
		i = 3;
		break;
	case 0xBEB: // stop
		i = 4;
		break;
	case 0xBEF: // record
		i = 5;
		break;
	case 0xBFB: // rewind
		i = 6;
		break;
	case 0xBFE: // blue
		i = 7;
		break;
	case 0xEBB: // guide | tv home
		i = 8;
		//i = 9;
		break;
	case 0xEBF: // louder
		i = 10;
		break;
	case 0xEEB: // quieter
		i = 11;
		break;
	case 0xEEF: // pause/play | back/?
		i = 12;
		//i = 13;
		break;
	case 0xEFB: // P+ | red | av
		i = 14;
		//i = 15;
		//i = 16
		break;
	case 0xEFE: // P- | green | right | audio off
		i = 17;
		//i = 18;
		//i = 19;
		//i = 20
		break;
	case 0xEFF: // yellow
		i = 21;
		break;
	case 0xFAB: // 0
		i = 22;
		break;
	case 0xFAF: // i-tv
		i = 23;
		break;
	case 0xFBB: // down | 9
		i = 24;
		//i = 25;
		break;
	case 0xFBE: // left | up | 8
		i = 26;
		//i = 27;
		//i = 28;
		break;
	case 0xFBF: // OK
		i = 29;
		break;
	case 0xFEB: // 5
		i = 30;
		break;
	case 0xFEE: // 4 | 6
		i = 31;
		//i = 32;
		break;
	case 0xFEF: // 7
		i = 33;
		break;
	case 0xFFA: // 2
		i = 34;
		break;
	case 0xFFB: // 3
		i = 35;
		break;
	case 0xFFE: // 1
		i = 36;
		break;
	default:
		i = 37;
		break;
	} // eof switch
	sprintf((char*)MSG, "No error. Key is: %s\r\n"
			, aKey[i]);
	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	// essential to get it working!!!
	// (concurrency or, it might be that UART2 is using TIM2...)
	HAL_Delay(50);
}

	//	uint8_t i = 0;
//	char aKey[34][14] = {{'\'','1','\'','\0'}
//	, {'\'','2','\'','\0'}
//	, {'\'','3','\'','\0'}
//	, {'\'','4','\'','\0'}
//	, {'\'','5','\'','\0'}
//	, {'\'','6','\'','\0'}
//	, {'\'','7','\'','\0'}
//	, {'\'','8','\'','\0'}
//	, {'\'','9','\'','\0'}
//	, {'\'','0','\'','\0'}
//	, {'\'','A','.','.','Z','/','1','.','.','0','\'','\0'}
//	, {'\'','P','+','\'','\0'}
//	, {'\'','P','-','\'','\0'}
//	, {'\'','g','i','d','s','\'','\0'}
//	, {'\'','r','a','d','i','o','\'','\0'}
//	, {'\'','t','e','r','u','g','/','?','\'','\0'}
//	, {'\'','m','e','n','u','\'','\0'}
//	, {'\'','t','v',' ','t','h','u','i','s','\'','\0'}
//	, {'\'','o','m','l','a','a','g','\'','\0'}
//	, {'\'','l','i','n','k','s','\'','\0'}
//	, {'\'','r','e','c','h','t','s','\'','\0'}
//	, {'\'','o','m','h','o','o','g','\'','\0'}
//	, {'\'','r','o','o','d','\'','\0'}
//	, {'\'','g','r','o','e','n','\'','\0'}
//	, {'\'','g','e','e','l','\'','\0'}
//	, {'\'','b','l','a','u','w','\'','\0'}
//	, {'\'','s','t','o','p','\'','\0'}
//	, {'\'','o','p','n','e','m','e','n','\'','\0'}
//	, {'\'','t','e','r','u','g','\'','\0'}
//	, {'\'','p','a','u','z','e','/','s','p','e','l','e','n','\0'}
//	, {'\'','v','o','o','r','u','i','t','\0'}
//	, {'\'','i','-','t','v','\0'}
//	, {'\'','O','K','\'','\0'}
//	, {'I','N','V','A','L','I','D','\0'}
//	};
//	switch (frame)
//	{
//	case 0x2EBFFFFFE: //'1'
//		i = 0;
//		break;
//	case 0x2EBFFFFFA: //'2'
//		i = 1;
//		break;
//	case 0x2EBFFFFFB: //'3'
//		i = 2;
//		break;
//	case 0x2EBFFFFEE: //'4'
//		i = 3;
//		break;
//	case 0x2EBFFFFEB: //'5'
//		i = 4;
//		break;
////	case 0x2EBFFFFEE: //'6' is the same as 4 ???
////		i = 5;
////		break;
//	case 0x2EBFFFFEF: //'7'
//		i = 6;
//		break;
//	case 0x2EBFFFFBE: //'8'
//		i = 7;
//		break;
//	case 0x2EBFFFFBB: //'9'
//		i = 8;
//		break;
//	case 0x2EBFFFFAB: //'0'
//		i = 9;
//		break;
////	case 0x2EBFFFFAB: //'A..Z/1..0' is the same as '0' ???
////		i = 10;
////		break;
//	case 0x2EBFFFEFB: //'P+'
//		i = 11;
//		break;
//	case 0x2EBFFFEFE: //'P-'
//		i = 12;
//		break;
//	case 0x2EBFFFEBB: //'gids'
//		i = 13;
//		break;
//	case 0x2EBFFFABF: //'radio'
//		i = 14;
//		break;
//	case 0x2EBFFFEEF: //'terug/?' is the same as gids ???
//		i = 15;
//		break;
//	case 0x2EBFFFAEF: //'menu'
//		i = 16;
//		break;
////	case 0x2EBFFFEBB: //'tv thuis' is the same as gids ???
////		i = 17;
////		break;
////	case 0x2EBFFFFBB: //'omlaag' is the same as 9 ???
////		i = 18;
////		break;
////	case 0x2EBFFFFBE: //'links' is the same as 8 ???
////		i = 19;
////		break;
////	case 0x2EBFFFEFE: //'rechts' is the same as P- ???
////		i = 20;
////		break;
////	case 0x2EBFFFFBE: //'omhoog' is the same as 8 ???
////		i = 21;
////		break;
////	case 0x2EBFFFEFB: //'rood' is the same as P+ ???
////		i = 22;
////		break;
////	case 0x2EBFFFEFE: //'groen' is the same as P- ???
////		i = 23;
////		break;
//	case 0x2EBFFFEFF: //'geel'
//		i = 24;
//		break;
//	case 0x2EBFFFBFE: //'blauw'
//		i = 25;
//		break;
//	case 0x2EBFFFBEB: //'stop'
//		i = 26;
//		break;
//	case 0x2EBFFFBEF: //'opnemen'
//		i = 27;
//		break;
//	case 0x2EBFFFBFB: //'terug'
//		i = 28;
//		break;
////	case 0x2EBFFFEEF: //'pauze/spelen' is the same as 'terug/?' ???
////		i = 29;
////		break;
//	case 0x2EBFFFBBB: //'vooruit'
//		i = 30;
//		break;
//	case 0x2EBFFFFAF: //'i-tv'
//		i = 31;
//		break;
//	case 0x2EBFFFFBF: //'OK'
//		i = 32;
//		break;
//	default: //INVALID
//		i = 33;
//		break;
//	}
//	sprintf((char*)MSG, "No error. Key is: %s\r\n"
//			, aKey[i]);
//	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
//	// essential to get it working!!!
//	  // (concurrency or, it might be that UART2 is using TIM2...)
//	HAL_Delay(50);
//}
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
