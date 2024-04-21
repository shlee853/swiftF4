/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define LED_GPIO_GREEN_R_Pin GPIO_PIN_13
#define LED_GPIO_GREEN_R_GPIO_Port GPIOC
#define LED_GPIO_RED_R_Pin GPIO_PIN_14
#define LED_GPIO_RED_R_GPIO_Port GPIOC
#define E_CS0_Pin GPIO_PIN_15
#define E_CS0_GPIO_Port GPIOC
#define NRF_EXINT_Pin GPIO_PIN_0
#define NRF_EXINT_GPIO_Port GPIOA
#define NRF_EXINT_EXTI_IRQn EXTI0_IRQn
#define USART2_TX_NRF_RX_Pin GPIO_PIN_2
#define USART2_TX_NRF_RX_GPIO_Port GPIOA
#define USART2_RX_NRF_TX_Pin GPIO_PIN_3
#define USART2_RX_NRF_TX_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOA
#define IMU_INT_EXTI_IRQn EXTI4_IRQn
#define MOTOR4_Pin GPIO_PIN_5
#define MOTOR4_GPIO_Port GPIOA
#define LED_GPIO_GREEN_L_Pin GPIO_PIN_6
#define LED_GPIO_GREEN_L_GPIO_Port GPIOA
#define LED_GPIO_RED_L_Pin GPIO_PIN_7
#define LED_GPIO_RED_L_GPIO_Port GPIOA
#define E_CS1_Pin GPIO_PIN_0
#define E_CS1_GPIO_Port GPIOB
#define MOTOR3_Pin GPIO_PIN_10
#define MOTOR3_GPIO_Port GPIOB
#define LED_GPIO_BLUE_L_Pin GPIO_PIN_12
#define LED_GPIO_BLUE_L_GPIO_Port GPIOB
#define E_SPI2_SCK_Pin GPIO_PIN_13
#define E_SPI2_SCK_GPIO_Port GPIOB
#define E_SPI2_MISO_Pin GPIO_PIN_14
#define E_SPI2_MISO_GPIO_Port GPIOB
#define E_SPI2_MOSI_Pin GPIO_PIN_15
#define E_SPI2_MOSI_GPIO_Port GPIOB
#define E_CS3_Pin GPIO_PIN_8
#define E_CS3_GPIO_Port GPIOA
#define E_CS4_Pin GPIO_PIN_9
#define E_CS4_GPIO_Port GPIOA
#define E_USART1_TX_Pin GPIO_PIN_15
#define E_USART1_TX_GPIO_Port GPIOA
#define E_USART1_RX_Pin GPIO_PIN_3
#define E_USART1_RX_GPIO_Port GPIOB
#define E_SDA_Pin GPIO_PIN_4
#define E_SDA_GPIO_Port GPIOB
#define MOTOR2_Pin GPIO_PIN_6
#define MOTOR2_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_7
#define MOTOR1_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
