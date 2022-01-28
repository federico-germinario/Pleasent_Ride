/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MQ_DATA_LENGTH 12
#define BASE_THRESHOLD 1.0
#define SLIDING_WINDOWS 7
#define COORDINATE_NUMBER 13

//MPU registers
#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43

#define FIFO_EN_REG 0x23
#define FIFO_R_W 0x74
#define USER_CTRL 0x6A
#define INT_ENABLE 0x38
#define CONFIG_REG 0x1A
#define FIFO_COUNTH 0x72
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t MPU_OK = 0;     //Check status MPU flag

//Fake gps structures
typedef struct {
	float longitude;
	float latitude;
}Coordinate;

Coordinate fake_gps[13];
uint8_t coordinate_index = 0;

//Sensor data structures
uint8_t mq_index = 0;
float mq_data[MQ_DATA_LENGTH];

////// MPU Variables
float Ax, Ay, Az, Gx, Gy, Gz = 0.0;
float Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW, Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW = 0.0;

uint8_t mpu_data[1024];
uint16_t mpu_index = 0;
uint8_t fall_detected = 0;

//Calibration offsets
float offset_gyroX = 6.725720;
float offset_gyroY = 8.554494;
float offset_gyroZ = 5.479887;

float offset_accelX = 0.889819;
float offset_accelY = 0.694547;
float offset_accelZ = 1 - 0.260066;

uint8_t check_fall = 0;
uint8_t check_fall_counter = 0;
uint16_t bad_quality_road_counter = 0;

////// END MPU Variables

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <errno.h>
#include <sys/stat.h>
#include "retarget_uart.h"

void MPU6050_Init(){
	uint8_t check, Data;
	uint8_t attempts = 0;

	// Check if the device is ready
	while(check != 104){
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
		if(attempts++ > 50){
			break;
		}
	}

	if(check == 104){ // If the device is ready

		// We write all zeros in register 0X6B to wake up the sensor
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// Let's put Gyro fs at 1KHz
		Data = 0x02;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);

		// DATA RATE = Gyroscope Output Rate (1 Khz) / (1 + SMPLRT_DIV (99)) ==> 10 Hz
		Data = 0x63;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Accelerometer configuration:
		// XA_ST = 0, YA_ST = 0, ZA_ST = 0, FS_SEL = 0 ==> Full Scale Range = +- 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Gyroscope configuration:
		// XG_ST = 0, YG_ST = 0, ZG_ST = 0, FS_SEL = 0 ==> Full Scale Range = +- 250 */s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

		// Enable write buffers for accel and gyro
		Data = 0x78;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);

		// Enable the buffer
		Data = 0x44;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, &Data, 1, 1000);

		// Enable interrupt with data read
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE, 1, &Data, 1, 1000);
	}
}

// Initialization of the circular buffer containing the coordinates generated by Google Eart.
void fake_gps_init(){

	fake_gps[0].longitude = 9.232127873367;
	fake_gps[0].latitude = 45.476947477674;

	fake_gps[1].longitude = 9.23132169559556;
	fake_gps[1].latitude = 45.4769356338021;

	fake_gps[2].longitude = 9.23027839341514;
	fake_gps[2].latitude = 45.4769302429461;

	fake_gps[3].longitude = 9.22848735956641;
	fake_gps[3].latitude = 45.4768485383483;

	fake_gps[4].longitude = 9.22811674289873;
	fake_gps[4].latitude = 45.4767904670821;

	fake_gps[5].longitude = 9.22542983923211;
    fake_gps[5].latitude = 45.4767755259172;

    fake_gps[6].longitude = 9.22554076080336;
    fake_gps[6].latitude = 45.477625700989;

    fake_gps[7].longitude = 9.22552155468664;
    fake_gps[7].latitude = 45.4788784152841;

    fake_gps[8].longitude = 9.22652317861608;
    fake_gps[8].latitude = 45.4792292326394;

    fake_gps[9].longitude = 9.22803205121175;
    fake_gps[9].latitude = 45.4791161835439;

    fake_gps[10].longitude = 9.22938409789064;
    fake_gps[10].latitude = 45.4790173241089;

    fake_gps[11].longitude = 9.22994266430211;
    fake_gps[11].latitude = 45.4782498335493;

    fake_gps[12].longitude = 9.22997546730472;
    fake_gps[12].latitude = 45.4788991756912;
}

Coordinate get_coordinate(){
	Coordinate c = fake_gps[coordinate_index];
	coordinate_index = (coordinate_index + 1) % COORDINATE_NUMBER;
	return c;
}

void reset_esp8266(){
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);

	HAL_GPIO_WritePin(ESP_Reset_GPIO_Port, ESP_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(ESP_Reset_GPIO_Port, ESP_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(200);

	HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn); //Clears the pending bit of an EXTI1 interrupt.
	__HAL_GPIO_EXTI_CLEAR_IT(ESP_Signal_Pin);

	//HAL_GPIO_WritePin(ESP_Signal_GPIO_Port, ESP_Signal_Pin, GPIO_PIN_RESET);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

//Function that returns the sum of a collection of a certain length
float float_sum(float* collection, uint8_t index) {
	float sum = 0.0;
	for(int i=0; i<index; i++){
		sum += collection[i];
	}
	return sum;
}


void fall_counter_increment(float gyro_value){
	if(check_fall){                   //Start counting after first bad spot
		check_fall_counter++;
		if(check_fall_counter>20){    //The fall can last 20 measurements
			if(gyro_value>30.0){      //Check user movements
				check_fall_counter=check_fall=0;
			}
		}
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
  memset(mq_data, 0, MQ_DATA_LENGTH*sizeof(float));
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart3); 	//Retarget printf to uart3
  fake_gps_init();

  MPU6050_Init();
  HAL_DMA_Init(&hdma_i2c1_rx);

  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_IT(&hadc1);

  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF); //Clear update interrupt flag
  HAL_TIM_Base_Start_IT(&htim2); 			//Start timer 30 s

  HAL_SuspendTick(); //The SysTick interrupt is disabled and then the Tick increment is suspended

  /*
   * Set SLEEPONEXIT bit of SCR register. When this bit is set, the processor re-enters SLEEP mode
   * when an interruption handling is over.
   */
  HAL_PWR_EnableSleepOnExit();

  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 29999;
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
  htim3.Init.Prescaler = 24999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  htim4.Init.Prescaler = 24999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8099;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP_Reset_GPIO_Port, ESP_Reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : ESP_Signal_Pin */
  GPIO_InitStruct.Pin = ESP_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP_Signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP_Reset_Pin */
  GPIO_InitStruct.Pin = ESP_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_DATA_RDY_Pin */
  GPIO_InitStruct.Pin = MPU_DATA_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_DATA_RDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2) {
		HAL_ResumeTick();
		DEBUG_PRINT(("[TIM2] 30 sec timer expired! SEND RESET TO ESP8266!\r\n\n"));

		// Check MPU
		if(!MPU_OK){
			MPU6050_Init();
		}

		reset_esp8266();

		HAL_SuspendTick();
	}

	if(htim->Instance == TIM4) {
		/*uint32_t cycles = 0;
		double timeCallback;

		DWT->CTRL |= 1;
		DWT->CYCCNT = 0;
		*/
		DEBUG_PRINT(("[TIM4] Reading inertial data from MPU-6050\r\n"));

		uint8_t countArr[2];
		uint16_t count=0;

		uint8_t Data = 0x0;

		//Stop fifo MPU-6050 writing before reading
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);

		//Number of bytes ready to read
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNTH, 1, countArr, 2, 1000);
		count = (uint16_t) (countArr[0] << 8 | countArr[1]);
		DEBUG_PRINT(("[TIM4] Fifo count: %d\r\n\n", count));

		if(count > 0 && count <= 1024) {
			mpu_index = count;
			Data = FIFO_R_W;

			//I2C master-slave comunication for reading fifo buffer
			HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &Data, 1, 1000);
			HAL_I2C_Master_Receive_DMA(&hi2c1, MPU6050_ADDR, &mpu_data[0], count);
		}
		else {
			//In case I cannot read the fifo buffer correctly, I initialise the device.
			MPU6050_Init();
		}

		/*cycles = DWT->CYCCNT;
		cycles--;

		timeCallback = ((double)cycles/HAL_RCC_GetHCLKFreq())*10e+3;
        printf("TIME (tim4 callback) =  %.2lf ms\n\r", timeCallback);
        */
	}

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


//Callback interrupt ESP8266
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ESP_Signal_Pin){

		/*uint32_t cycles = 0;
		double timeCallback;

		DWT->CTRL |= 1;
		DWT->CYCCNT = 0;
		*/
		DEBUG_PRINT(("[ESP_SIGNAL] Sending data to ESP8266\r\n"));
		uint16_t road_quality = 0;

		float mq_mean = float_sum(mq_data, mq_index)/mq_index;
		Coordinate c = get_coordinate();

		if(bad_quality_road_counter <= 10){
			road_quality = 0; //Excellent road quality
		}else if(bad_quality_road_counter > 10 && bad_quality_road_counter <= 20){
			road_quality = 1; //Decent road quality
		}else if(bad_quality_road_counter > 20){
			road_quality = 2; //Bad road quality
		}

		bad_quality_road_counter = 0;

		DEBUG_PRINT(("[ESP_SIGNAL] Data: Longitude=%f, Latitude=%f, PPM = %f, Road_quality = %d, Fall_detected = %d\r\n\n", c.longitude, c.latitude, mq_mean, road_quality, fall_detected));

		char line[60];

		snprintf(line, sizeof(line), "%f,%f,%f,%d,%d\n", c.longitude, c.latitude, mq_mean, road_quality, fall_detected);

		fall_detected=0;
		mq_index=0;

		//Sending data via USART to ESP8266
		HAL_UART_Transmit(&huart2, (uint8_t*) (line), strlen(line), 1000);

		__HAL_GPIO_EXTI_CLEAR_IT(EXTI1_IRQn);
		HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);

		HAL_NVIC_DisableIRQ(EXTI1_IRQn);

		/*cycles = DWT->CYCCNT;
		cycles--;

		timeCallback = ((double)cycles/HAL_RCC_GetHCLKFreq())*10e+3;
		printf("TIME (ESP_Signal) =  %.2lf ms\n\r", timeCallback);
		*/
	}

	if(GPIO_Pin == MPU_DATA_RDY_Pin){

		//Periodic timer start for mpu reading
		DEBUG_PRINT(("[MPU_DATA_RDY_Pin] MPU-6050 fifo buffer starts to fill up\r\n\n"));
		__HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF);
		HAL_TIM_Base_Start_IT(&htim4); //Start 8 sec timer
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc){

	uint16_t rawValue; float ppm; float v;

	rawValue = HAL_ADC_GetValue(hadc);

	v = ((float)rawValue) / 4095 * 4668;
	ppm = ((v - 320.0) / 0.65) + 400;
	//DEBUG_PRINT(("rawValue: %hu\r\n", rawValue));
	//DEBUG_PRINT(("v: %f\r\n", v));
	DEBUG_PRINT(("[HAL_ADC_ConvCpltCallback] PPM: %f\r\n\n", ppm));
	mq_data[mq_index] = ppm;
	mq_index = (mq_index + 1) % MQ_DATA_LENGTH;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c){
	/*uint32_t cycles = 0;
	double timeCallback;

	DWT->CTRL |= 1;
	DWT->CYCCNT = 0;
	*/

	DEBUG_PRINT(("[HAL_I2C_MasterRxCpltCallback] Receive DMA callback\r\n"));
	MPU_OK = 1;

	/// Reactivate write buffer

	uint8_t Data = 0x78;
	//Restart writing inertial data to fifo buffer
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);
	Data = 0x44;
	//I clean the fifo buffer and reactivate its writing
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, &Data, 1, 1000);

	float means[SLIDING_WINDOWS];
	float temp_sums[SLIDING_WINDOWS+1];
	float accel_vectors[mpu_index/12], gyro_vectors[mpu_index/12];
	uint16_t accel_vector_index=0, gyro_vector_index = 0;
	float accel_sum = 0;
	uint8_t sum_counter = 0, sum_index=0;


	//Compute accelerations vectors
	for(int i=0; i<mpu_index; i+=12){
		Accel_X_RAW = (int16_t)(mpu_data[i] << 8 | mpu_data[i+1]);
		Accel_Y_RAW = (int16_t)(mpu_data[i+2] << 8 | mpu_data[i+3]);
		Accel_Z_RAW = (int16_t)(mpu_data[i+4] << 8 | mpu_data[i+5]);

		Ax = Accel_X_RAW/16384.0 + offset_accelX;
		Ay = Accel_Y_RAW/16384.0 + offset_accelY;
		Az = Accel_Z_RAW/16384.0 + offset_accelZ;

		float raw_amp = sqrt(pow(Ax, 2)+pow(Ay, 2)+pow(Az, 2));
		accel_vectors[accel_vector_index++] = raw_amp;
		accel_sum = accel_sum + raw_amp;
		sum_counter++;
		if (sum_counter == 10){
			temp_sums[sum_index++] = accel_sum;
			sum_counter = 0;
			accel_sum = 0;
		}

		Gyro_X_RAW = (int16_t)(mpu_data[i+6] << 8 | mpu_data[i+6+1]);
		Gyro_Y_RAW = (int16_t)(mpu_data[i+6+2] << 8 | mpu_data[i+6+3]);
		Gyro_Z_RAW = (int16_t)(mpu_data[i+6+4] << 8 | mpu_data[i+6+5]);

		Gx = Gyro_X_RAW/131.0 + offset_gyroX;
		Gy = Gyro_Y_RAW/131.0 + offset_gyroY;
		Gz = Gyro_Z_RAW/131.0 + offset_gyroZ;

		float gyro_vector = sqrt(pow(Gx, 2)+pow(Gy, 2)+pow(Gz, 2));
		gyro_vectors[gyro_vector_index++] = gyro_vector;

		//DEBUG_PRINT(("Burst #%d: linear acceleration axis x: %f g, y: %f g, z: %f g\r\n", i/12, Ax, Ay, Az));
		//DEBUG_PRINT(("Gyro exis x: %f °/s, y: %f °/s, z: %f °/s \tgyro_vector: %f\r\n", Gx, Gy, Gz, gyro_vector));
	}

	//Calculate sliding windows means
	for(int i=0; i<SLIDING_WINDOWS; i++){
			means[i]=(temp_sums[i]+temp_sums[i+1])/20;
		}

	//Road quality end fall detection algorithm
	for (int i = 0; i < 7; ++i) {
		for(int j=0; j<20; j++){
			if(j<10 && i<6){
				fall_counter_increment(gyro_vectors[10*i+j]);
			}
			if(i==6){
				fall_counter_increment(gyro_vectors[10*i+j]);
			}
			if(check_fall_counter > 60) {
				fall_detected=1;
				check_fall_counter=check_fall=0;
			}
			float threshold = BASE_THRESHOLD - gyro_vectors[10*i+j]/250.0;
			float difference = abs(accel_vectors[10*i+j] - means[i]);
			if(difference > threshold) {
				//DEBUG_PRINT(("Bad point to the measurement nr. %d\r\n", 10*i+j));
				bad_quality_road_counter++;
				check_fall=1;
			}
		}
	}

	/*cycles = DWT->CYCCNT;
	cycles--;

	timeCallback = ((double)cycles/HAL_RCC_GetHCLKFreq())*10e+3;

	printf("TIME (HAL_I2C_MasterRxCpltCallback) =  %.2lf ms\n\r", timeCallback);
	*/
	DEBUG_PRINT(("[HAL_I2C_MasterRxCpltCallback] Bad points calculated: %d\r\n", bad_quality_road_counter));
	DEBUG_PRINT(("[HAL_I2C_MasterRxCpltCallback] Fall_detected: %d\r\n\n", fall_detected));

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
