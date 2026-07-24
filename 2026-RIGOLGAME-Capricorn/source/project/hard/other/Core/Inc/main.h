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
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOC
#define Hong1_Pin GPIO_PIN_2
#define Hong1_GPIO_Port GPIOC
#define Hong2_Pin GPIO_PIN_3
#define Hong2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define EN1_OUTA_Pin GPIO_PIN_6
#define EN1_OUTA_GPIO_Port GPIOA
#define EN1_OUTB_Pin GPIO_PIN_7
#define EN1_OUTB_GPIO_Port GPIOA
#define HUI8_Pin GPIO_PIN_4
#define HUI8_GPIO_Port GPIOC
#define HUI7_Pin GPIO_PIN_5
#define HUI7_GPIO_Port GPIOC
#define HUI6_Pin GPIO_PIN_0
#define HUI6_GPIO_Port GPIOB
#define HUI5_Pin GPIO_PIN_1
#define HUI5_GPIO_Port GPIOB
#define HUI4_Pin GPIO_PIN_12
#define HUI4_GPIO_Port GPIOB
#define HUI3_Pin GPIO_PIN_13
#define HUI3_GPIO_Port GPIOB
#define HUI2_Pin GPIO_PIN_14
#define HUI2_GPIO_Port GPIOB
#define DUO_A_Pin GPIO_PIN_6
#define DUO_A_GPIO_Port GPIOC
#define DUO_B_Pin GPIO_PIN_7
#define DUO_B_GPIO_Port GPIOC
#define DUO_C_Pin GPIO_PIN_8
#define DUO_C_GPIO_Port GPIOC
#define DUO_D_Pin GPIO_PIN_9
#define DUO_D_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define YA_SCK_Pin GPIO_PIN_11
#define YA_SCK_GPIO_Port GPIOA
#define YA_DT_Pin GPIO_PIN_12
#define YA_DT_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_15
#define PWMA_GPIO_Port GPIOA
#define Chao_Echo_Pin GPIO_PIN_12
#define Chao_Echo_GPIO_Port GPIOC
#define Chao_Trig_Pin GPIO_PIN_2
#define Chao_Trig_GPIO_Port GPIOD
#define PWMB_Pin GPIO_PIN_3
#define PWMB_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOB
#define KEY4_Pin GPIO_PIN_5
#define KEY4_GPIO_Port GPIOB
#define EN2_OUTA_Pin GPIO_PIN_6
#define EN2_OUTA_GPIO_Port GPIOB
#define EN2_OUTB_Pin GPIO_PIN_7
#define EN2_OUTB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
