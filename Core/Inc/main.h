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
#include "stm32h7xx_hal.h"

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
void FDCAN_DISABLE_INTERRUPTS(void);
void FDCAN_ENABLE_INTERRUPTS(void);
void CLEAR_OUTPUT(void);
void Accelerometer_reset(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SB_FP_CTRL_Pin GPIO_PIN_4
#define SB_FP_CTRL_GPIO_Port GPIOF
#define SB_DR_CTRL_Pin GPIO_PIN_5
#define SB_DR_CTRL_GPIO_Port GPIOF
#define SB_BP2_CTRL_Pin GPIO_PIN_6
#define SB_BP2_CTRL_GPIO_Port GPIOF
#define CAN_LED_Pin GPIO_PIN_5
#define CAN_LED_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_0
#define GREEN_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOB
#define POWER_GOOD_Pin GPIO_PIN_15
#define POWER_GOOD_GPIO_Port GPIOB
#define SB_BP1_CTRL_Pin GPIO_PIN_6
#define SB_BP1_CTRL_GPIO_Port GPIOG
#define CH4_Pin GPIO_PIN_0
#define CH4_GPIO_Port GPIOD
#define CH2_Pin GPIO_PIN_1
#define CH2_GPIO_Port GPIOD
#define CH2_EXTI_IRQn EXTI1_IRQn
#define YELLOW_LED_Pin GPIO_PIN_1
#define YELLOW_LED_GPIO_Port GPIOE
#define SB_BP3_CTRL_Pin GPIO_PIN_0
#define SB_BP3_CTRL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
