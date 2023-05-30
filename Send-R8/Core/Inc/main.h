/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define EBYTE1_AUX_Pin GPIO_PIN_13
#define EBYTE1_AUX_GPIO_Port GPIOC
#define EBYTE1_M0_Pin GPIO_PIN_14
#define EBYTE1_M0_GPIO_Port GPIOC
#define EBYTE1_M1_Pin GPIO_PIN_15
#define EBYTE1_M1_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_2
#define SPI_CS_GPIO_Port GPIOC
#define SPI_DC_Pin GPIO_PIN_3
#define SPI_DC_GPIO_Port GPIOC
#define SPI_RES_Pin GPIO_PIN_4
#define SPI_RES_GPIO_Port GPIOC
#define KEY_5_Pin GPIO_PIN_10
#define KEY_5_GPIO_Port GPIOB
#define KEY_5_EXTI_IRQn EXTI15_10_IRQn
#define KEY_6_Pin GPIO_PIN_11
#define KEY_6_GPIO_Port GPIOB
#define KEY_6_EXTI_IRQn EXTI15_10_IRQn
#define KEY_7_Pin GPIO_PIN_12
#define KEY_7_GPIO_Port GPIOB
#define KEY_7_EXTI_IRQn EXTI15_10_IRQn
#define KEY_8_Pin GPIO_PIN_13
#define KEY_8_GPIO_Port GPIOB
#define KEY_8_EXTI_IRQn EXTI15_10_IRQn
#define LED_1_Pin GPIO_PIN_14
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOB
#define SW_1_1_Pin GPIO_PIN_6
#define SW_1_1_GPIO_Port GPIOC
#define SW_1_2_Pin GPIO_PIN_7
#define SW_1_2_GPIO_Port GPIOC
#define SW_2_1_Pin GPIO_PIN_8
#define SW_2_1_GPIO_Port GPIOC
#define SW_2_2_Pin GPIO_PIN_9
#define SW_2_2_GPIO_Port GPIOC
#define EBYTE0_AUX_Pin GPIO_PIN_8
#define EBYTE0_AUX_GPIO_Port GPIOA
#define EBYTE0_M0_Pin GPIO_PIN_11
#define EBYTE0_M0_GPIO_Port GPIOA
#define EBYTE0_M1_Pin GPIO_PIN_12
#define EBYTE0_M1_GPIO_Port GPIOA
#define KEY_ROCKER_Pin GPIO_PIN_15
#define KEY_ROCKER_GPIO_Port GPIOA
#define KEY_ROCKER_EXTI_IRQn EXTI15_10_IRQn
#define KEY_1_Pin GPIO_PIN_6
#define KEY_1_GPIO_Port GPIOB
#define KEY_1_EXTI_IRQn EXTI9_5_IRQn
#define KEY_2_Pin GPIO_PIN_7
#define KEY_2_GPIO_Port GPIOB
#define KEY_2_EXTI_IRQn EXTI9_5_IRQn
#define KEY_3_Pin GPIO_PIN_8
#define KEY_3_GPIO_Port GPIOB
#define KEY_3_EXTI_IRQn EXTI9_5_IRQn
#define KEY_4_Pin GPIO_PIN_9
#define KEY_4_GPIO_Port GPIOB
#define KEY_4_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
