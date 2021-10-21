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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if !defined(OS_USE_SEMIHOSTING)
#include <errno.h>
#include <sys/stat.h>
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

UART_HandleTypeDef* gHuart;

void  RetargetInit(UART_HandleTypeDef *huart){
	gHuart=huart;

	setvbuf(stdout, NULL, _IONBF, 0);
}

int _isatty(int fd){
	if(fd>=STDIN_FILENO && fd<=STDERR_FILENO){
		return 1;
	}

	errno = EBADF;
	return 0;
}

int _write(int fd, char* ptr, int len) {
	HAL_StatusTypeDef hstatus;

	if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
		hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
		if (hstatus == HAL_OK)
			return len;
		else
			return EIO;
	}
	errno = EBADF;
	return -1;
}

int _close(int fd) {
	if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
		return 0;

	errno = EBADF;
	return -1;
}

int _lseek(int fd, int ptr, int dir) {
	(void) fd;
	(void) ptr;
	(void) dir;

	errno = EBADF;
	return -1;
}

int _read(int fd, char* ptr, int len) {
	HAL_StatusTypeDef hstatus;

	if (fd == STDIN_FILENO) {
		hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
		if (hstatus == HAL_OK)
			return 1;
		else
			return EIO;
	}

	errno = EBADF;
	return -1;
}

int _fstat(int fd, struct stat* st) {
	if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
		st->st_mode = S_IFCHR;
		return 0;
	}

	errno = EBADF;
	return 0;

}

#endif

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //pulizia terminale
   //HAL_UART_Transmit(&huart2, (uint8_t*)"\027[2J", strlen("\027[2J"), HAL_MAX_DELAY);

   RetargetInit(&huart2);

   //setvbuf(stdout, NULL, _IONBF, 0);
   printf("Init done\n\r");
   //setvbuf(stdout, NULL, _IONBF, 0);

   char buf[100];
   memset(buf, 0, sizeof(buf));

   /////RST

   HAL_UART_Transmit(&huart3, (uint8_t*) ("AT+RST\r\n"), strlen("AT+RST\r\n"), 1000);

   //HAL_UART_Receive(&huart3, (uint8_t *) buf, sizeof(buf), 1000);
   //printf("reset: %s\r\n", buf);
   printf("RESET\r\n");
   //setvbuf(stdout, NULL, _IONBF, 0);
   HAL_Delay(3000);

   /////AT

   HAL_UART_Transmit(&huart3, (uint8_t*) ("AT\r\n"), strlen("AT\r\n"), 1000);

   HAL_UART_Receive(&huart3, (uint8_t *) (buf), sizeof(buf), 1000);
   HAL_UART_Transmit(&huart2, (uint8_t*) (buf), sizeof(buf), 1000);

   printf("AT: %s\r\n", buf);
   //setvbuf(stdout, NULL, _IONBF, 0);
   memset(buf, 0, sizeof(buf));
   HAL_Delay(2000);

   /////CWMODE

   HAL_UART_Transmit(&huart3, (uint8_t*) ("AT+CWMODE=1\r\n"), strlen("AT+CWMODE=1\r\n"), 1000);
   HAL_UART_Receive(&huart3, (uint8_t *) buf, sizeof(buf), 1000);

   printf("AT+CWMODE=1: %s\r\n", buf);
   //setvbuf(stdout, NULL, _IONBF, 0);
   memset(buf, 0, sizeof(buf));

   ///CWJAP

   char data[60];
   memset(data, 0, sizeof(data));
   sprintf (data, "AT+CWJAP=\"jackhuai\",\"laborra2\"\r\n");
   //sprintf (data, "AT+CWJAP=\"iPhone di Federico\",\"12345678\"\r\n");
   HAL_UART_Transmit(&huart3, (uint8_t *) (data), sizeof(data), 1000);

   HAL_UART_Receive(&huart3, (uint8_t *) buf, sizeof(buf), 1000);
   printf("AT+CWJAP: %s\r\n", buf);
   //setvbuf(stdout, NULL, _IONBF, 0);
   memset(buf, 0, sizeof(buf));
   ///CIPMUX
   HAL_UART_Transmit(&huart3, (uint8_t*) ("AT+CIPMUX=0\r\n"), strlen("AT+CIPMUX=0\r\n"), 1000);
   HAL_UART_Receive(&huart3, (uint8_t *) buf, sizeof(buf), 1000);
   printf("AT+CIPMUX=0: %s\r\n", buf);
   //setvbuf(stdout, NULL, _IONBF, 0);
   memset(buf, 0, sizeof(buf));

   printf("Setup end\n\r");
   //setvbuf(stdout, NULL, _IONBF, 0);

   HAL_Delay(5000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t counter = 0;
  while (1){

	  HAL_UART_Transmit(&huart3, (uint8_t*) ("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n"), strlen("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n"), 1000);
	  HAL_UART_Receive(&huart3, (uint8_t *) buf, sizeof(buf), 1000);
	  printf("%s\r\n", buf);

	  char local_buf[100] = {0};
	  char local_buf2[30] = {0};
	  HAL_UART_Receive(&huart3, (uint8_t *) buf, sizeof(buf), 3000);
	  printf("%s\r\n", buf);
	  memset(buf, 0, sizeof(buf));

	  sprintf (local_buf, "GET /update?api_key=%s&field%d=%u\r\n", "MK77FV2ZF1VMIUYQ", 1, counter);
	  counter = counter +1;
	  int len = strlen (local_buf);

	  sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	  HAL_UART_Transmit(&huart3, (uint8_t *) local_buf2, sizeof(local_buf2), 1000);

	  HAL_Delay(1000);

	  HAL_UART_Transmit(&huart3, (uint8_t *) local_buf, sizeof(local_buf), 1000);

	  HAL_Delay(1000);

	  HAL_UART_Transmit(&huart3, (uint8_t*) ("AT+CIPCLOSE\r\n"), strlen("AT+CIPCLOSE\r\n"), 1000);
	  HAL_Delay(22000);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
