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
//#define DEBUG_MODE
#ifdef __cplusplus
extern "C" {
#endif
#ifdef DEBUG_MODE
   #define UDS_DELAY 10
#else
   #define UDS_DELAY 100
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
void UDS_READ_ERRORS(uint8_t status_byte);
unsigned long SeedToKey(unsigned long seed,unsigned char rnd);
void store_CANframeRX(uint8_t framenum,uint8_t* data, size_t length);
void Send_Result(void);
void EnterSecurityAccess(void);
void store_CANframeTX(uint8_t framenum,uint8_t* data, size_t length,uint16_t ID);
void ClearDTC(uint8_t OutputFrameStartPosition);
uint8_t FDCAN_Get_FIFO_Put_index(bool FIFOnbr);
void Send_Request(uint8_t Request_to_send,uint8_t StartPositionInOutput);
void Write_VIN(bool VIN_to_WRITE);
void CheckACUConfiguration(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SB_FP_CTRL_Pin GPIO_PIN_13
#define SB_FP_CTRL_GPIO_Port GPIOC
#define SB_BP2_CTRL_Pin GPIO_PIN_2
#define SB_BP2_CTRL_GPIO_Port GPIOE
#define SB_BP1_CTRL_Pin GPIO_PIN_3
#define SB_BP1_CTRL_GPIO_Port GPIOE
#define SBRS_CTRL_Pin GPIO_PIN_4
#define SBRS_CTRL_GPIO_Port GPIOE
#define SB_DR_CTRL_Pin GPIO_PIN_1
#define SB_DR_CTRL_GPIO_Port GPIOE
#define SQUIB_SW_CTRL_Pin GPIO_PIN_5
#define SQUIB_SW_CTRL_GPIO_Port GPIOE
#define SB_BP3_CTRL_Pin GPIO_PIN_0
#define SB_BP3_CTRL_GPIO_Port GPIOE
#define CAN_LED_Pin GPIO_PIN_10
#define CAN_LED_GPIO_Port GPIOA
#define POWER_GOOD_Pin GPIO_PIN_0
#define POWER_GOOD_GPIO_Port GPIOC
#define POWER_GOOD_EXTI_IRQn EXTI0_IRQn
#define CH11_Pin GPIO_PIN_14
#define CH11_GPIO_Port GPIOB
#define CH11_EXTI_IRQn EXTI15_10_IRQn
#define CH0_Pin GPIO_PIN_12
#define CH0_GPIO_Port GPIOE
#define CH0_EXTI_IRQn EXTI15_10_IRQn
#define CH2_Pin GPIO_PIN_10
#define CH2_GPIO_Port GPIOB
#define CH2_EXTI_IRQn EXTI15_10_IRQn
#define CH8_Pin GPIO_PIN_9
#define CH8_GPIO_Port GPIOD
#define CH8_EXTI_IRQn EXTI9_5_IRQn
#define CH1_Pin GPIO_PIN_13
#define CH1_GPIO_Port GPIOE
#define CH1_EXTI_IRQn EXTI15_10_IRQn
#define CH3_Pin GPIO_PIN_11
#define CH3_GPIO_Port GPIOB
#define CH3_EXTI_IRQn EXTI15_10_IRQn
#define CH7_Pin GPIO_PIN_8
#define CH7_GPIO_Port GPIOD
#define CH7_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
