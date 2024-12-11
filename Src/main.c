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
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "DHT11.h"
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
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//phan bien xu ly nhan command
uint8_t cbyte;
char command_string[16]={0};
uint8_t command_index=0;

char command[5];
char numOfParam=0;
struct params{
	char first[5];
	char second[5];
	char third[5];
};
struct params command_param;

/************************

//PHAN DINH NGHIA COMMAND NHAN
 * <COMMAND>:<SO_LUONG_PARAM>:<PARAM1>:<PARAM2>:<PARAM3>
161 - WATERING
162 - WATER TIME SETTING
163 - WATER SCHEDULE
165 - SYSTEM TIME
//PHAN DINH NGHIA COMMAND GUI DI
 * <COMMAND>:<GIA_TRI>
200 - LIGHT INFO
201 - SOIL HUMID INFO
202 - DONE WATERING
203 - ENVI TEMP INFO
204 - ENVI HUMID INFO

*************************/

//BIEN SU DUNG BEN TRONG STM32
RTC_AlarmTypeDef sAlarm = {0};

//WATERING - 161
bool watering=0;
uint32_t start_watering;
uint32_t time_now;

//THOI GIAN TUOI - 162
int thoi_gian_tuoi=1000;

//SCHEDULE - 163
int hour=5;
int minute=45;
int sys_hour=0;
int sys_minute=0;
//LIGHTING - INFO - 200
int dosang;

//SOIL WATER - INFO - 201
int doam;

//ENVI TEMP AND HUMID - INFO - 202 & 203
int tempe=0;
int humid=0;
DHT11_InitTypeDef dht;
DHT11_StatusTypeDef err;

//SENDING INFO
ADC_ChannelConfTypeDef sConfig = {0};
uint8_t sendTime=0;
char infoBuffer[20]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//ham mapping
float inverse_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (out_max - (out_max - out_min) * (x - in_min) / (in_max - in_min));
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (out_max - (out_max - out_min) * (x - in_min) / (in_max - in_min));
}

//phan xu ly command tu app
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(cbyte != '\r')
	{
		command_string[command_index]=cbyte;
		command_index++;
	}else{

		char *command_part;

		command_part = strtok(command_string, ":");
		if (command_part != NULL) {
			strncpy(command, command_part, sizeof(command) - 1);
		}


		command_part = strtok(NULL,":");
		if (command_part != NULL) {
			numOfParam = command_part[0];
		}

		//XU LY LENH CO 2 KY TU
		if(numOfParam == '2') {
		    command_part = strtok(NULL,  ":");
		    if (command_part != NULL) {
		    	strncpy(command_param.first, command_part, sizeof(command_param.first) - 1);
		    }

		    command_part = strtok(NULL,  ":");
		    if (command_part != NULL) {
		    	strncpy(command_param.second, command_part, sizeof(command_param.second) - 1);
		    }
		//XU LY LENH CO 1 KY TU
		}else if(numOfParam == '1') {
			command_part = strtok(NULL,  ":");
			if (command_part != NULL) {
				strncpy(command_param.first, command_part, sizeof(command_param.first) - 1);
			}
		}

		//PHAN XU LY LOGIC

		/*
		 * 161 - TIN HIEU TUOI CAY
		 */
		if(strcmp(command,"161")==0)
		{
			if(watering!=1)
			{
				watering=1;
				start_watering=HAL_GetTick();

			}
		}
		/*
		 * 162 - TIN HIEU CAI DAT TUOI BAO LAU
		 */
		else if(strcmp(command,"162")==0)
		{
			thoi_gian_tuoi=atoi(command_param.first);
		}
		/*
		* 163 - TIN HIEU CAI DAT THOI GIAN TUOI
		*/
		else if(strcmp(command,"163")==0)
		{
			hour=atoi(command_param.first);
			minute=atoi(command_param.second);
			//Thay doi thoi gian hen gio
			sAlarm.AlarmTime.Hours = hour;
			sAlarm.AlarmTime.Minutes = minute;
			sAlarm.AlarmTime.Seconds = 00;
			sAlarm.Alarm = RTC_ALARM_A;
			if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
			{
			  Error_Handler();
			}
		}
		/*
		* 165 - TIN HIEU CAI DAT THOI GIAN HE THONG
		*/
		else if(strcmp(command,"165")==0)
		{
			sys_hour=atoi(command_param.first);
			sys_minute=atoi(command_param.second);
			//cau hinh cho phep thay doi thoi gian he thong
			 HAL_PWR_EnableBkUpAccess();
			// Disable Write Protection
			__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

			// Allow access to RTC time/date registers
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);

			//Thay doi thoi gian hen gio
			RTC_TimeTypeDef sTime = {0};
			sTime.Hours = sys_hour;
			sTime.Minutes = sys_minute;
			sTime.Seconds = 0;
			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
			{
			  Error_Handler();
			}

		    // Enable Write Protection back
		    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
		}

		//phan reset cac bien du lieu
		command_index=0;
		for(int i=0; i<16; i++)
		{
			command_string[i]=0;
		}
		numOfParam=0;
		for(int i=0; i<5; i++)
		{
			command[i]=0;
			command_param.first[i]=0;
			command_param.second[i]=0;
			command_param.third[i]=0;
		}

	}
	HAL_UART_Receive_IT(&huart1, &cbyte, 1);
}

//PHAN XU LY ALARM
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	if(watering!=1)
	{
		watering=1;
		start_watering=HAL_GetTick();
	}
}

//PHAN XU LY DOC VA GUI DU LIEU CAM BIEN VOI TIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	sendTime++;
	if(sendTime==3)
	{
		//phan chon chanel
		sConfig.Channel=ADC_CHANNEL_0;
		sConfig.Rank=1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		//phan nhan du lieu
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		dosang= HAL_ADC_GetValue(&hadc1);
		dosang=inverse_map(dosang, 0, 4096, 0, 100);
//		//phan gui du lieu di
//		/*
//		 	 * 200 - LIGHTING INFO
//		*/
		sprintf(infoBuffer, "200:%d\r", dosang);
		HAL_UART_Transmit(&huart1, (uint8_t *)infoBuffer, strlen(infoBuffer), 100);
		memset(infoBuffer, 0, sizeof(infoBuffer));
	}else if(sendTime==4)
	{
		//phan chon chanel
		sConfig.Channel=ADC_CHANNEL_1;
		sConfig.Rank=1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		//phan nhan du lieu
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		doam= HAL_ADC_GetValue(&hadc1);
		doam=map(doam, 0, 4096, 0, 100);
		//phan gui du lieu di
//		/*
//		 	 * 201 - SOIL WATER INFO
//		*/
		sprintf(infoBuffer, "201:%d\r", doam);
		HAL_UART_Transmit(&huart1, (uint8_t *)infoBuffer, strlen(infoBuffer), 100);
		memset(infoBuffer, 0, sizeof(infoBuffer));
	}else if(sendTime==5)
	{
		//phan gui du lieu di
//		/*
//		 	 * 203 - ENVI TEMP INFO
//		*/
		sprintf(infoBuffer, "203:%d\r", tempe);
		HAL_UART_Transmit(&huart1, (uint8_t *)infoBuffer, strlen(infoBuffer), 100);
		memset(infoBuffer, 0, sizeof(infoBuffer));
	}else if(sendTime==7)
	{
		//phan gui du lieu di
//		/*
//		 	 * 204 - ENVI HUMID INFO
//		*/
		sprintf(infoBuffer, "204:%d\r", humid);
		HAL_UART_Transmit(&huart1, (uint8_t *)infoBuffer, strlen(infoBuffer), 100);
		memset(infoBuffer, 0, sizeof(infoBuffer));
		sendTime=0;
	}
}

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
  HAL_Delay(2000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &cbyte, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_DHT11_Init(&dht, GPIOB, GPIO_PIN_14, &htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 //dieu khien may bom - COMMAND 160
	 if(watering==1)
	 {
	  	time_now=HAL_GetTick();
	  	if(time_now - start_watering > thoi_gian_tuoi)
	  	{
	  		watering=false;
	  		//GUI TIN HIEU SAU KHI TUOI XONG
	  		sprintf(infoBuffer, "202:0\r");
	  		HAL_UART_Transmit(&huart1, (uint8_t *)infoBuffer, strlen(infoBuffer), 100);
	  		memset(infoBuffer, 0, sizeof(infoBuffer));
	  	}
	 }
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, watering);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//test
	 err = HAL_DHT11_ReadData(&dht);
	 if(err != DHT11_OK) {
		 continue;
	 }
	 tempe=(int)dht.Temperature;
	 humid=(int)dht.Humidity;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 5;
  sTime.Minutes = 41;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 5;
  sAlarm.AlarmTime.Minutes = 45;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
