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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct tagFRAME
{
    const char soh;
    uint16_t cmnd;
    const uint8_t stx;
    int16_t payload;
    const char etx;
    const char etb;
    const char eot;
} FRAME, *PFRAME;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOH					1
#define STX					2
#define ETX					3
#define ETB					23
#define EOT					4
#define LEN_FRAME			9
#define DELAY_4HZ_SERIAL	125
#define BUFFER_MAX_SERIAL	512

// command serial communication
// read command
#define RD_REG_CNFG			33600 // address to approach two bytes with 12 bits data
#define RD_REG_T_LO			33601 // address to approach two bytes with 12/13 bits data
#define RD_REG_T_HI			33602 // address to approach two bytes with 12/13 bits data
#define RD_REG_TEMP			33603 // address to approach two bytes with 12/13 bits data (with 13 bits data LSB=1 in second byte)
// write command
#define WR_REG_CNFG			33610 // address to approach two bytes with 12 bits data
#define WR_REG_T_LO			33611 // address to approach two bytes with 12/13 bits data
#define WR_REG_T_HI			33612 // address to approach two bytes with 12/13 bits data

//#define BUFFER_MAX 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t TMP102_ADDR = 0x48 << 1;
static const uint8_t REG_TEMP = 0x00;
static const uint8_t REG_CNFG = 0x01;
static const uint8_t REG_T_LO = 0x02;
static const uint8_t REG_T_HI = 0x03;

// this is the way to initialize the attributes, which are constant,
// in the struct, and also on the fly
// initialize the attributes, which are not constant, in the struct
FRAME frame = { SOH, 0, STX, 0, ETX, ETB, EOT };
// initially this slave will receive from its master
bool bReceive = true;
char chBuffer[BUFFER_MAX_SERIAL] = { 0 };

//uint8_t pData[BUFFER_MAX];
//bool bReceive = true;
//uint16_t iCommand = 0;
HAL_StatusTypeDef ret;
uint8_t buf_i2c[12];
//bool bReadTemp102 = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (bReceive)
	  {
		  HAL_UART_Receive_IT(&huart2
				  , (uint8_t*)chBuffer
				  , LEN_FRAME
				  );
		  HAL_Delay(DELAY_4HZ_SERIAL);
	  }
	  if (!bReceive)
	  {
		  switch (frame.cmnd)
		  {
		  case RD_REG_CNFG:
		  case RD_REG_T_LO:
		  case RD_REG_T_HI:
		  {
			  if (frame.cmnd == RD_REG_CNFG) buf_i2c[0] = REG_CNFG;
			  if (frame.cmnd == RD_REG_T_LO) buf_i2c[0] = REG_T_LO;
			  if (frame.cmnd == RD_REG_T_HI) buf_i2c[0] = REG_T_HI;
			  ret = HAL_I2C_Master_Transmit(&hi2c1
					  , TMP102_ADDR
					  , buf_i2c
					  , 1
					  , HAL_MAX_DELAY
			  );
			  ret = HAL_I2C_Master_Receive(&hi2c1
					  , TMP102_ADDR
					  , buf_i2c
					  , 2
					  , HAL_MAX_DELAY
			  );
			  // the returned value has to go into the chBuffer
			  chBuffer[4] = buf_i2c[0];
			  chBuffer[5] = buf_i2c[1];
			  break;
		  } // eof RD_REG_CNFG
		  case RD_REG_TEMP:
		  {
			  buf_i2c[0] = REG_TEMP;
			  ret = HAL_I2C_Master_Transmit(&hi2c1
					  , TMP102_ADDR
					  , buf_i2c
					  , 1
					  , HAL_MAX_DELAY
			  );
			  ret = HAL_I2C_Master_Receive(&hi2c1
					  , TMP102_ADDR
					  , buf_i2c
					  , 2
					  , HAL_MAX_DELAY
			  );
			  // the returned value has to go into the chBuffer
			  chBuffer[4] = buf_i2c[0];
			  chBuffer[5] = buf_i2c[1];
			  // check if alert bit is set
			  buf_i2c[0] = REG_CNFG;
			  ret = HAL_I2C_Master_Transmit(&hi2c1
					  , TMP102_ADDR
					  , buf_i2c
					  , 1
					  , HAL_MAX_DELAY
			  );
			  ret = HAL_I2C_Master_Receive(&hi2c1
					  , TMP102_ADDR
					  , buf_i2c
					  , 2
					  , HAL_MAX_DELAY
			  );
			  uint8_t alert_bit = (buf_i2c[1] & 0x20) >> 4;
			  chBuffer[5] |= alert_bit;
			  break;
		  } // eof RD_REG_TEMP
		  } // eof switch
		  HAL_UART_Transmit_IT(&huart2
				  , (uint8_t*)chBuffer
				  , LEN_FRAME
				  );
		  HAL_Delay(DELAY_4HZ_SERIAL);
	  }
//	  if (bReceive)
//	  {
//		  HAL_UART_Receive_IT(&huart2
//				  , pData
//				  , 5);
//		  HAL_Delay(125);
//	  }
//	  if (!bReceive)
//	  {
//		  switch (iCommand)
//		  {
//		  case 33600: // read
//		  case 33610: // write
//		  {
//			  // tell TMP102 that we want to read from, or write to,
//			  // the configuration register
//			  buf_i2c[0] = REG_CNFG;
//			  break;
//		  } // eof 33600 | 33610
//		  case 33601: // read
//		  case 33611: // write
//		  {
//			  // tell TMP102 that we want to read from, or write to,
//			  // the temp-low register
//			  buf_i2c[0] = REG_T_LO;
//			  break;
//		  } // eof 33601 | 33611
//		  case 33602: // read
//		  case 33612: // write
//		  {
//			  // tell TMP102 that we want to read from, or write to,
//			  // the temp-high register
//			  buf_i2c[0] = REG_T_HI;
//			  break;
//		  } // eof 33602 | 33612
//		  case 33603:
//		  {
//			  // tell TMP102 that we want to read from the temperature register
//			  buf_i2c[0] = REG_TEMP;
//			  break;
//		  } // eof 33603
//		  default:
//			  break;
//		  } // eof switch
//		  ret = HAL_I2C_Master_Transmit(&hi2c1
//				  , TMP102_ADDR
//				  , buf_i2c
//				  , 1
//				  , HAL_MAX_DELAY
//		  );
//		  if (ret == HAL_OK)
//		  {
//			  if (bReadTemp102)
//			  {
//				  // insert delay here, to allow for conversion
//				  HAL_Delay(50);
//				  ret = HAL_I2C_Master_Receive(&hi2c1
//						  , TMP102_ADDR
//						  , buf_i2c
//						  , 2
//						  , HAL_MAX_DELAY
//				  );
//				  if (ret == HAL_OK)
//				  {
//					  pData[0] = buf_i2c[0];
//					  pData[1] = buf_i2c[1];
//				  }
//				  else
//				  {
//					  pData[0] = 0;
//					  pData[1] = 0;
//				  }
//				  HAL_UART_Transmit_IT(&huart2
//						  , pData
//						  , 2);
//				  // insert second delay, to complete a 125 ms delay
//				  HAL_Delay(75);
//			  }
//			  if (!bReadTemp102)
//			  {
//				  // write to the chosen register
//			  }
//		  }
//	  }
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//////////////////////////////////////////////////////////////////////////////
//****************************************************************************
//*                     HAL_UART_RxCpltCallback
//****************************************************************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	frame.cmnd = (chBuffer[1] << 8) | chBuffer[2];
	bReceive = false;
//	iCommand = atoi((const char*)pData);
//	if (iCommand >= 33600 && iCommand <= 33603)
//	{
//		bReadTemp102 = true;
//	}
//	if (iCommand >= 33610 && iCommand <= 33612)
//	{
//		bReadTemp102 = false;
//	}
//	bReceive = false;
}
//****************************************************************************
//*                     HAL_UART_TxCpltCallback
//****************************************************************************
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	bReceive = true;
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
