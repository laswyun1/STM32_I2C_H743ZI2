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
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_DOUBLE_DATA_NUM 		10
#define I2C_FLOAT_DATA_NUM 			20
#define I2C_INT32_DATA_NUM 			10
#define I2C_UINT32_DATA_NUM 		10
#define I2C_INT16_DATA_NUM 			10
#define I2C_UINT16_DATA_NUM 		10
#define I2C_INT8_DATA_NUM 			10
#define I2C_UINT8_DATA_NUM 			10

#define DATA_TOTAL_BYTE 			100		// Should be Larger than NEEDED
#define I2C_TOTAL_DATA_NUM			(I2C_DOUBLE_DATA_NUM + I2C_FLOAT_DATA_NUM + I2C_INT32_DATA_NUM + I2C_UINT32_DATA_NUM + I2C_INT16_DATA_NUM + I2C_UINT16_DATA_NUM + I2C_INT8_DATA_NUM + I2C_UINT8_DATA_NUM)

#define SLAVE_ADDR					(0x7f << 1)


typedef enum _IOIF_I2CDatatType_t {
	I2C_DOUBLE = 1,
	I2C_FLOAT,
	I2C_INT32,
	I2C_UINT32,
	I2C_INT16,
	I2C_UINT16,
	I2C_INT8,
	I2C_UINT8
} IOIF_I2CDataType_t;

typedef struct _IOIF_I2CObj_t {
	double doubleI2CData[I2C_DOUBLE_DATA_NUM];
	float floatI2CData[I2C_FLOAT_DATA_NUM];
	int32_t int32I2CData[I2C_INT32_DATA_NUM];
	uint32_t uint32I2CData[I2C_UINT32_DATA_NUM];
	int16_t int16I2CData[I2C_INT16_DATA_NUM];
	uint16_t uint16I2CData[I2C_UINT16_DATA_NUM];
	int8_t int8I2CData[I2C_INT8_DATA_NUM];
	uint8_t uint8I2CData[I2C_UINT8_DATA_NUM];

	uint8_t doubleIndex;
	uint8_t floatIndex;
	uint8_t int32Index;
	uint8_t uint32Index;
	uint8_t int16Index;
	uint8_t uint16Index;
	uint8_t int8Index;
	uint8_t uint8Index;

	uint8_t totalDataNum;
	uint16_t totalDataByte;

	uint8_t dataTypeArray[I2C_TOTAL_DATA_NUM];

	uint8_t ParsedI2CByteData[DATA_TOTAL_BYTE];
} IOIF_I2CObj_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* For DMA I2C Communication (with DMA) */
static uint8_t i2c2CommDmaRxBuff[DATA_TOTAL_BYTE] __attribute__((section(".i2c2RxBuff"))) = {0};

uint8_t rxByteDataArray[DATA_TOTAL_BYTE] = {0};

/* Receive */
void byteValueToRealValue(void* realData, uint8_t* byteData)
{
	static uint16_t rxDataIndex = 0;
	static uint8_t rxDataNum = 0;
	static uint8_t firstRead = 0;
	static uint8_t totalDataNum = 0;

	/* Check Total Data Number */
	if (firstRead == 0){
		totalDataNum = byteData[rxDataIndex];
		rxDataIndex++;
		firstRead = 1;
	}

	/* Parsing */
	if (byteData[rxDataIndex] == I2C_DOUBLE){
		memcpy((double*)realData, &byteData[rxDataIndex+1], sizeof(double));
		rxDataIndex += 9;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_FLOAT){
		memcpy((float*)realData, &byteData[rxDataIndex+1], sizeof(float));
		rxDataIndex += 5;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_INT32){
		memcpy((int32_t*)realData, &byteData[rxDataIndex+1], sizeof(int32_t));
		rxDataIndex += 5;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_UINT32){
		memcpy((uint32_t*)realData, &byteData[rxDataIndex+1], sizeof(uint32_t));
		rxDataIndex += 5;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_INT16){
		memcpy((int16_t*)realData, &byteData[rxDataIndex+1], sizeof(int16_t));
		rxDataIndex += 3;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_UINT16){
		memcpy((uint16_t*)realData, &byteData[rxDataIndex+1], sizeof(uint16_t));
		rxDataIndex += 3;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_INT8){
		memcpy((int8_t*)realData, &byteData[rxDataIndex+1], sizeof(int8_t));
		rxDataIndex += 2;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == I2C_UINT8){
		memcpy((uint8_t*)realData, &byteData[rxDataIndex+1], sizeof(uint8_t));
		rxDataIndex += 2;
		rxDataNum++;
	}
	else {
		rxDataIndex = 0;
		rxDataNum = 0;
		firstRead = 0;
		return;
	}

	/* Optimization case */
	if (rxDataIndex == DATA_TOTAL_BYTE){
		rxDataIndex = 0;
		rxDataNum = 0;
		firstRead = 0;
		return;
	}

	if (rxDataNum == totalDataNum){
		rxDataIndex = 0;
		rxDataNum = 0;
		firstRead = 0;
		return;
	}
}


/* For Received Values */
float data1;
float data2;
float data3;
float data4;
float data5;
float data6;
float data7;
float data8;
float data9;
float data10;
float data11;
float data12;
float data13;



void settingI2CReceivedData(uint8_t* byteData)
{
	byteValueToRealValue(&data1, byteData);
	byteValueToRealValue(&data2, byteData);
	byteValueToRealValue(&data3, byteData);
	byteValueToRealValue(&data4, byteData);
	byteValueToRealValue(&data5, byteData);
	byteValueToRealValue(&data6, byteData);
	byteValueToRealValue(&data7, byteData);
//	byteValueToRealValue(&data8, byteData);
//	byteValueToRealValue(&data9, byteData);
//	byteValueToRealValue(&data10, byteData);
//	byteValueToRealValue(&data11, byteData);
//	byteValueToRealValue(&data12, byteData);
//	byteValueToRealValue(&data13, byteData);
}


uint32_t timeElapsed = 0;

/* Receive */
uint8_t IOIF_ReceiveI2CData(void)
{
	memset(rxByteDataArray, 0, DATA_TOTAL_BYTE);
	uint8_t rxDebug = HAL_I2C_Slave_Receive_DMA(&hi2c2, i2c2CommDmaRxBuff, DATA_TOTAL_BYTE);
	memcpy(rxByteDataArray, i2c2CommDmaRxBuff, DATA_TOTAL_BYTE);
	settingI2CReceivedData(rxByteDataArray);

	return rxDebug;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_ETH_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM2 : I2C2(Slave) - Receive */
	if (htim == &htim2){
		/* Loop Start Time Check */
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

		/* Used Function */
		IOIF_ReceiveI2CData();
		///////////////////////

		timeElapsed = DWT->CYCCNT / 64;	// in microsecond
	}
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
