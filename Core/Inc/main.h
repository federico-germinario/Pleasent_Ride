/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP_Signal_Pin GPIO_PIN_1
#define ESP_Signal_GPIO_Port GPIOC
#define ESP_Signal_EXTI_IRQn EXTI1_IRQn
#define USART2_ESP_TX_Pin GPIO_PIN_2
#define USART2_ESP_TX_GPIO_Port GPIOA
#define USART2_ESP_RX_Pin GPIO_PIN_3
#define USART2_ESP_RX_GPIO_Port GPIOA
#define ESP_Reset_Pin GPIO_PIN_8
#define ESP_Reset_GPIO_Port GPIOE
#define USART3_DEBUG_TX_Pin GPIO_PIN_10
#define USART3_DEBUG_TX_GPIO_Port GPIOB
#define USART3_DEBUG_RX_Pin GPIO_PIN_11
#define USART3_DEBUG_RX_GPIO_Port GPIOB
#define MPU_DATA_RDY_Pin GPIO_PIN_11
#define MPU_DATA_RDY_GPIO_Port GPIOD
#define MPU_DATA_RDY_EXTI_IRQn EXTI15_10_IRQn
#define MPU6050_I2C1_SCL_Pin GPIO_PIN_6
#define MPU6050_I2C1_SCL_GPIO_Port GPIOB
#define MPU6050_I2C1_SDA_Pin GPIO_PIN_7
#define MPU6050_I2C1_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
