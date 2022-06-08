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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

#if !defined(ARDUINO_ARCH_STM32)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

#endif

#if defined(ARDUINO_BLUEPILL_F103C8)

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define TestIn_Pin GPIO_PIN_14
#define TestIn_GPIO_Port GPIOC
#define TestOut_Pin GPIO_PIN_15
#define TestOut_GPIO_Port GPIOC
#define Dbg4_Pin GPIO_PIN_0
#define Dbg4_GPIO_Port GPIOB
#define SPI_SEL_Pin GPIO_PIN_12
#define SPI_SEL_GPIO_Port GPIOB
#define Sel_Pin GPIO_PIN_8
#define Sel_GPIO_Port GPIOA
#define Dbg5_Pin GPIO_PIN_15
#define Dbg5_GPIO_Port GPIOA
#define Dbg0_Pin GPIO_PIN_4
#define Dbg0_GPIO_Port GPIOB
#define Dbg1_Pin GPIO_PIN_5
#define Dbg1_GPIO_Port GPIOB
#define Dbg2_Pin GPIO_PIN_8
#define Dbg2_GPIO_Port GPIOB
#define Dbg3_Pin GPIO_PIN_9
#define Dbg3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#else

#undef __MAIN_H
#if defined(ARDUINO_ARCH_STM32)
#include "mainN.h"
#else
#include "../../MonitorCPPN/Inc/main.h"
#endif  /* ARDUINO_ARCH_STM32 */

#endif	 /* ARDUINO_BLUEPILL_F103C8 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
