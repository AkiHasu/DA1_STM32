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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */ // Hoặc nằm ngay trên đây

/* USER CODE END PFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DHT22_PIN_Pin GPIO_PIN_0
#define DHT22_PIN_GPIO_Port GPIOA
#define RCWL_PIN_Pin GPIO_PIN_1
#define RCWL_PIN_GPIO_Port GPIOA
#define TFT_RST_Pin GPIO_PIN_2
#define TFT_RST_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_3
#define TFT_CS_GPIO_Port GPIOA
#define TFT_RS_Pin GPIO_PIN_4
#define TFT_RS_GPIO_Port GPIOA
#define LED_PIN_Pin GPIO_PIN_5
#define LED_PIN_GPIO_Port GPIOC
#define MFRC522_RST_Pin GPIO_PIN_0
#define MFRC522_RST_GPIO_Port GPIOB
#define MFRC522_IRQ_Pin GPIO_PIN_1
#define MFRC522_IRQ_GPIO_Port GPIOB
#define MFRC522_SDA_Pin GPIO_PIN_11
#define MFRC522_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
