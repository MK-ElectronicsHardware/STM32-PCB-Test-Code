/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  *
  * Solo MK2 Interface board Test Firmware
  *Author : Matthew King
  *
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
#include "string.h"
#include "stdio.h"
#include "PressureSense.h"
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

//I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
static void base_test_run();
void base_test_read_adc();
void UART1_transmit_mode();
void UART2_transmit_mode();
void UART3_transmit_mode();
void UART1_receive_mode();
void UART2_receive_mode();
void UART3_receive_mode();
//static void base_test_read_adc();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//digital inputs
uint8_t encoder=0;
uint8_t digital_input_1=0;
uint8_t digital_input_2=0;

//adc varaiables
uint16_t thermistor_analogue=0;
uint16_t supply_mon_analogue=0;
uint16_t flow_analogue=0;

volatile uint16_t adc_result[3];//dma result
const int adc_channel_count = sizeof(adc_result)/sizeof(adc_result[0]);
volatile int adc_conversion_complete=0;//set by callback

//sensor
int32_t pressure_sensor=0;
uint32_t g_raw_pressure_sensor=0;
uint32_t g_raw_temperature_sensor=0; // added by ST 16/01/2023 for debugging the raw PTX sensors raw temp reading
//led counter
uint8_t status_led_counter=0;

uint8_t rx_data_uart1;
uint8_t rx_data_uart2;
uint8_t rx_data_uart3;

void base_test_run()
{

	/***********START ENCODER*****************/
	encoder = (uint8_t)GPIOB->IDR & 0x0F;//read the encoder (lower 4 bits of port B)
	/***********END ENCODER*****************/

	/***********START READ DIGITAL INPUTS*****************/
	digital_input_1 = HAL_GPIO_ReadPin(Dig_In_1_GPIO_Port, Dig_In_1_Pin);//read digital input 1
	digital_input_2 = HAL_GPIO_ReadPin(Dig_In_2_GPIO_Port, Dig_In_2_Pin);//read digital input 2

	if(digital_input_1)
	{
		HAL_GPIO_WritePin(LED_Flow_OK_GPIO_Port, LED_Flow_OK_Pin, GPIO_PIN_RESET);//turn ON flow ok led
		HAL_GPIO_WritePin(Pump_En_GPIO_Port, Pump_En_Pin, GPIO_PIN_RESET);//turn pump OFF as  logic zero turns on vcc iso
	}
	else
	{
		HAL_GPIO_WritePin(LED_Flow_OK_GPIO_Port, LED_Flow_OK_Pin, GPIO_PIN_SET);//turn OFF flow ok led
		HAL_GPIO_WritePin(Pump_En_GPIO_Port, Pump_En_Pin, GPIO_PIN_RESET);//turn pump OFF as  logic zero turns on vcc iso
	}

	if(digital_input_2)
	{
		HAL_GPIO_WritePin(LED_GFC_OK_GPIO_Port, LED_GFC_OK_Pin, GPIO_PIN_RESET);//turn ON GFC led
	}
	else
	{
		HAL_GPIO_WritePin(LED_GFC_OK_GPIO_Port, LED_GFC_OK_Pin, GPIO_PIN_SET);//turn OFF GFC led
	}

	//every 10 counts toggle the status led
	if(status_led_counter==10)
	{
		HAL_GPIO_TogglePin(LED_Status_GPIO_Port, LED_Status_Pin);
		status_led_counter=0;
	}
	status_led_counter++;
	/***********END LEDS*****************/

	/*****************START ANALOGUES****************************/
	//uint32_t analogue_inputs[3];
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_result, adc_channel_count); // start adc in DMA mode abondoned dma dosent work
//	while(adc_conversion_complete==0)
//	{
//
//	}
//	adc_conversion_complete=0;//reset conversion complete

	base_test_read_adc();//update the ADC values

	pressure_sensor = PressureSense_ReadPressure();

	//thermistor_analogue = (uint16_t)adc_result[0];
	//supply_mon_analogue = (uint16_t)adc_result[1];
	//flow_analogue = (uint16_t)adc_result[2];
	/*****************END ANALOGUES****************************/

	/*******************START UART1***************************/
	char buffer[200]={0};
	int length=0;

// all lines below here were commented out so that stephen could just display the raw PTX sensor pressure and temp readings
//  this was done as a debugging excercise as on solo Mk2 his ptx readings were jumpy turns out the RTOS was returning 1 byte rathern than 2 bytes so rsoloution was very blocky
//	length+= sprintf(buffer, "Thermistor %d ", thermistor_analogue);
//	length+= sprintf(buffer + strlen(buffer), "Supply %d ", supply_mon_analogue);
//	length+= sprintf(buffer + strlen(buffer), "Flow %d ", flow_analogue);
//	length+= sprintf(buffer + strlen(buffer), "Encoder %d ", encoder);
//	length+= sprintf(buffer + strlen(buffer), "Dig1 %d ", digital_input_1);
//	length+= sprintf(buffer + strlen(buffer), "Dig2 %d ", digital_input_2);
///	length+= sprintf(buffer + strlen(buffer), "PTX %ld ", pressure_sensor); //this is not transmitted as the temp reading returns 0 so( compensated & ptxraw = 0)
	g_raw_pressure_sensor = (g_raw_pressure_sensor/6586); //scales the ptx sensor to 100 mbar 6658586/
	length+= sprintf(buffer + strlen(buffer), "PTX %ld ", g_raw_pressure_sensor);
	length+= sprintf(buffer + strlen(buffer), "RT %ld ", g_raw_temperature_sensor);
	length+= sprintf(buffer + strlen(buffer), "\r\n");





	//HAL_UART_Transmit(&huart1,(uint8_t *)buffer,sizeof(buffer),1000);// Sending in normal mode
	UART1_transmit_mode();//set to receive mode
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,length,1000);// transmit buffer data
	UART1_receive_mode();//set to receive mode




	// all commented out not needed uarts 2 and uarts 3 tranmit charater recieved in interrupt mode see 'echo'
	//	uint8_t data_uart2[] = "UART2\r\n";
//	uint8_t data_uart3[] = "UART3\r\n";
//	//send UART2 data
//	HAL_GPIO_WritePin(USART2_RS485_En_GPIO_Port, USART2_RS485_En_Pin, GPIO_PIN_SET);//set UART2 in transmit mode
//	HAL_GPIO_WritePin(USART3_RS485_En_GPIO_Port, USART3_RS485_En_Pin, GPIO_PIN_RESET);//set UART3 in receive mode
//	HAL_UART_Transmit(&huart2,(uint8_t *)data_uart2,sizeof(data_uart2),1000);// transmit uart2 data
//
//	HAL_Delay(100);//delay 100ms;
//
//	//send UART3 data
//	HAL_GPIO_WritePin(USART2_RS485_En_GPIO_Port, USART2_RS485_En_Pin, GPIO_PIN_RESET);//set UART2 in receive mode
//	HAL_GPIO_WritePin(USART3_RS485_En_GPIO_Port, USART3_RS485_En_Pin, GPIO_PIN_SET);//set UART3 in transmit mode
//	HAL_UART_Transmit(&huart3,(uint8_t *)data_uart3,sizeof(data_uart3),1000);// transmit uart3 data





	/*******************END UART1***************************/
} // end of void base test run



void base_test_read_adc()
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	//select channel 1
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	{
		thermistor_analogue = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop (&hadc1);

	//select channel 2
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	{
		supply_mon_analogue = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop (&hadc1);

	//select channel 3
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	{
		flow_analogue = HAL_ADC_GetValue(&hadc1);
	}

	HAL_ADC_Stop (&hadc1);
}

void UART1_transmit_mode()
{
	HAL_GPIO_WritePin(USART1_RS485_En_GPIO_Port, USART1_RS485_En_Pin, GPIO_PIN_SET);//set UART1 in transmit mode
}


void UART1_receive_mode()
{
	HAL_GPIO_WritePin(USART1_RS485_En_GPIO_Port, USART1_RS485_En_Pin, GPIO_PIN_RESET);//set UART1 in receive mode
}

void UART2_transmit_mode()
{
	//HAL_GPIO_WritePin(USART2_RS485_En_GPIO_Port, USART1_RS485_En_Pin, GPIO_PIN_SET);//set UART2 in transmit mode
	HAL_GPIO_WritePin(USART2_RS485_En_GPIO_Port, USART2_RS485_En_Pin, GPIO_PIN_SET);//set UART2 in transmit mode
}


void UART2_receive_mode()
{
	HAL_GPIO_WritePin(USART2_RS485_En_GPIO_Port, USART2_RS485_En_Pin, GPIO_PIN_RESET);//set UART2 in receive mode
}

void UART3_transmit_mode()
{
	HAL_GPIO_WritePin(USART3_RS485_En_GPIO_Port, USART3_RS485_En_Pin, GPIO_PIN_SET);//set UART3 in transmit mode
}


void UART3_receive_mode()
{
HAL_GPIO_WritePin(USART3_RS485_En_GPIO_Port, USART3_RS485_En_Pin, GPIO_PIN_RESET);//set UART3 in receive mode
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_conversion_complete=1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		 //Transmit one byte with 100 ms timeout this is an eco send an 'a' prints back a b // eco uart1
		UART1_transmit_mode();
		rx_data_uart1+=1;
		HAL_UART_Transmit(&huart1, &rx_data_uart1, 1, 100);

		UART1_receive_mode();
		//Receive one byte in interrupt mode
		HAL_UART_Receive_IT(&huart1, &rx_data_uart1, 1);
	}

	if (huart->Instance == USART2)                // eco uart 2 send an 'a' prints back 'b'
	{
		UART2_transmit_mode();
		//HAL_Delay(10);
		rx_data_uart2+=1;
		 //Transmit one byte with 100 ms timeout
		 HAL_UART_Transmit(&huart2, &rx_data_uart2, 1, 100);

		 UART2_receive_mode();
		//Receive one byte in interrupt mode
		HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1);
	}

	if (huart->Instance == USART3)               //eco uart 3 send an 'a' prints back b
	{
		UART3_transmit_mode();
		//HAL_Delay(10);
		rx_data_uart3+=1;
		 //Transmit one byte with 100 ms timeout
		 HAL_UART_Transmit(&huart3, &rx_data_uart3, 1, 100);

		 UART3_receive_mode();
		//Receive one byte in interrupt mode
		HAL_UART_Receive_IT(&huart3, &rx_data_uart3, 1);
	}




 // HAL_UART_Receive_IT(&huart2, Rx_data, 4);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  PressureSense_Initialise();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Receive one byte in interrupt mode
  HAL_UART_Receive_IT(&huart1, &rx_data_uart1, 1);
  HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1);
  HAL_UART_Receive_IT(&huart3, &rx_data_uart3, 1);

  while (1)
  {
	  base_test_run();
	  HAL_Delay(100);//delay 100ms;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Spare_Pin_6_Pin|USART2_RS485_En_Pin|Spare_Pin_1_Pin|USART1_RS485_En_Pin
                          |Spare_Pin_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Flow_OK_Pin|LED_GFC_OK_Pin|LED_Status_Pin|Pump_En_Pin
                          |USART3_RS485_En_Pin|Spare_Pin_3_Pin|Spare_Pin_4_Pin|Spare_Pin_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Spare_Pin_6_Pin USART2_RS485_En_Pin Spare_Pin_1_Pin USART1_RS485_En_Pin
                           Spare_Pin_2_Pin */
  GPIO_InitStruct.Pin = Spare_Pin_6_Pin|USART2_RS485_En_Pin|Spare_Pin_1_Pin|USART1_RS485_En_Pin
                          |Spare_Pin_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Enc_Bit_1_Pin Enc_Bit_2_Pin Enc_Bit_4_Pin Enc_BIt_8_Pin */
  GPIO_InitStruct.Pin = Enc_Bit_1_Pin|Enc_Bit_2_Pin|Enc_Bit_4_Pin|Enc_BIt_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Flow_OK_Pin LED_GFC_OK_Pin LED_Status_Pin Pump_En_Pin
                           USART3_RS485_En_Pin_Pin Spare_Pin_3_Pin Spare_Pin_4_Pin Spare_Pin_5_Pin */
  GPIO_InitStruct.Pin = LED_Flow_OK_Pin|LED_GFC_OK_Pin|LED_Status_Pin|Pump_En_Pin
                          |USART3_RS485_En_Pin|Spare_Pin_3_Pin|Spare_Pin_4_Pin|Spare_Pin_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Dig_In_1_Pin Dig_In_2_Pin */
  GPIO_InitStruct.Pin = Dig_In_1_Pin|Dig_In_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
