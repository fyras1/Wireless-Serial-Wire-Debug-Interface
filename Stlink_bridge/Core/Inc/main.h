/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f7xx_hal.h"
#include "commontask.h"
#include "error_code.h"
#include "cmsis_os.h"
#include "swd_slavetask.h"
#include "swd_mastertask.h"
#include "firmware_ver.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t txBuff[9];
uint8_t rxBuff[9];
notificationStruct queue[3];
//uint8_t front=0;
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
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB

#define SWD_SLAVE_DATA_Pin GPIO_PIN_9
#define SWD_SLAVE_DATA_GPIO_Port GPIOE

#define SWD_SLAVE_CLK_Pin GPIO_PIN_11
#define SWD_SLAVE_CLK_GPIO_Port GPIOE

#define SWD_SLAVE_CLK_EXTI_IRQn EXTI15_10_IRQn

#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB

#define SWD_MASTER_CLk_Pin GPIO_PIN_12
#define SWD_MASTER_CLk_GPIO_Port GPIOD

#define SWD_MASTER_DATA_Pin GPIO_PIN_13
#define SWD_MASTER_DATA_GPIO_Port GPIOD

#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
