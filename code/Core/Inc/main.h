/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define FLASH_INIT  0x08000000   //This is the page zero of our flash
#define DATA_SPACE  8            //Samples between each saved element
#define PAGE_SECTOR 2048         //Page size
#define DATA_PAGE   31           //Page that will store our data
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
#define A1_Pin LL_GPIO_PIN_0
#define A1_GPIO_Port GPIOA
#define A2_Pin LL_GPIO_PIN_1
#define A2_GPIO_Port GPIOA
#define A3_Pin LL_GPIO_PIN_4
#define A3_GPIO_Port GPIOA
#define A4_Pin LL_GPIO_PIN_5
#define A4_GPIO_Port GPIOA
#define A5_Pin LL_GPIO_PIN_6
#define A5_GPIO_Port GPIOA
#define A6_Pin LL_GPIO_PIN_7
#define A6_GPIO_Port GPIOA
#define K1_Pin LL_GPIO_PIN_0
#define K1_GPIO_Port GPIOB
#define K2_Pin LL_GPIO_PIN_1
#define K2_GPIO_Port GPIOB
#define K3_Pin LL_GPIO_PIN_2
#define K3_GPIO_Port GPIOB
#define A7_Pin LL_GPIO_PIN_8
#define A7_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define A8_Pin LL_GPIO_PIN_15
#define A8_GPIO_Port GPIOA
#define K4_Pin LL_GPIO_PIN_3
#define K4_GPIO_Port GPIOB
#define K5_Pin LL_GPIO_PIN_4
#define K5_GPIO_Port GPIOB
#define K6_Pin LL_GPIO_PIN_5
#define K6_GPIO_Port GPIOB
#define K7_Pin LL_GPIO_PIN_6
#define K7_GPIO_Port GPIOB
#define K8_Pin LL_GPIO_PIN_7
#define K8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
