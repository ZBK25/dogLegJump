/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOE
#define PWMA_Pin GPIO_PIN_5
#define PWMA_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_6
#define PWMB_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_8
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI9_5_IRQn
#define KEY1_Pin GPIO_PIN_9
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI9_5_IRQn
#define KEY2_Pin GPIO_PIN_10
#define KEY2_GPIO_Port GPIOE
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define KEY3_Pin GPIO_PIN_11
#define KEY3_GPIO_Port GPIOE
#define KEY3_EXTI_IRQn EXTI15_10_IRQn
#define LED0_Pin GPIO_PIN_12
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOE
#define PWMD_Pin GPIO_PIN_14
#define PWMD_GPIO_Port GPIOB
#define PWMC_Pin GPIO_PIN_15
#define PWMC_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_10
#define DIN2_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_11
#define DIN1_GPIO_Port GPIOC
#define CIN1_Pin GPIO_PIN_12
#define CIN1_GPIO_Port GPIOC
#define CIN2_Pin GPIO_PIN_0
#define CIN2_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
