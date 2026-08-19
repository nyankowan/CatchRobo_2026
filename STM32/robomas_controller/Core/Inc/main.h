/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define UPPER_ARM_R_LIMIT_Pin GPIO_PIN_2
#define UPPER_ARM_R_LIMIT_GPIO_Port GPIOC
#define UPPER_ARM_DEG_UNDER_LIMIT_Pin GPIO_PIN_3
#define UPPER_ARM_DEG_UNDER_LIMIT_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Status_LED_Pin GPIO_PIN_5
#define Status_LED_GPIO_Port GPIOA
#define UPPER_ARM_DEG_OVER_LIMIT_Pin GPIO_PIN_4
#define UPPER_ARM_DEG_OVER_LIMIT_GPIO_Port GPIOC
#define LOWER_ARM_R_LIMIT_Pin GPIO_PIN_5
#define LOWER_ARM_R_LIMIT_GPIO_Port GPIOC
#define ROBOMAS_CAN_RX_Pin GPIO_PIN_12
#define ROBOMAS_CAN_RX_GPIO_Port GPIOB
#define ROBOMAS_CAN_TX_Pin GPIO_PIN_13
#define ROBOMAS_CAN_TX_GPIO_Port GPIOB
#define COMMAND_CAN_RX_Pin GPIO_PIN_11
#define COMMAND_CAN_RX_GPIO_Port GPIOA
#define COMMAND_CAN_TX_Pin GPIO_PIN_12
#define COMMAND_CAN_TX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LOWER_ARM_DEG_UNDER_LIMIT_Pin GPIO_PIN_10
#define LOWER_ARM_DEG_UNDER_LIMIT_GPIO_Port GPIOC
#define LOWER_ARM_DEG_OVER_LIMIT_Pin GPIO_PIN_11
#define LOWER_ARM_DEG_OVER_LIMIT_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
