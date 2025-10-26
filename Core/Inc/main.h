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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define vbas_Pin GPIO_PIN_15
#define vbas_GPIO_Port GPIOC
#define Line_sensor1_Pin GPIO_PIN_0
#define Line_sensor1_GPIO_Port GPIOC
#define Line_sensor2_Pin GPIO_PIN_1
#define Line_sensor2_GPIO_Port GPIOC
#define Line_sensor3_Pin GPIO_PIN_2
#define Line_sensor3_GPIO_Port GPIOC
#define Line_sensor4_Pin GPIO_PIN_3
#define Line_sensor4_GPIO_Port GPIOC
#define Line_sensor5_Pin GPIO_PIN_0
#define Line_sensor5_GPIO_Port GPIOA
#define Line_sensor6_Pin GPIO_PIN_1
#define Line_sensor6_GPIO_Port GPIOA
#define Line_sensor7_Pin GPIO_PIN_2
#define Line_sensor7_GPIO_Port GPIOA
#define Line_sensor8_Pin GPIO_PIN_3
#define Line_sensor8_GPIO_Port GPIOA
#define Line_sensor9_Pin GPIO_PIN_4
#define Line_sensor9_GPIO_Port GPIOA
#define Line_sensor10_Pin GPIO_PIN_5
#define Line_sensor10_GPIO_Port GPIOA
#define Line_sensor11_Pin GPIO_PIN_6
#define Line_sensor11_GPIO_Port GPIOA
#define Line_sensor12_Pin GPIO_PIN_7
#define Line_sensor12_GPIO_Port GPIOA
#define Line_sensor13_Pin GPIO_PIN_4
#define Line_sensor13_GPIO_Port GPIOC
#define Line_sensor14_Pin GPIO_PIN_5
#define Line_sensor14_GPIO_Port GPIOC
#define Line_sensor15_Pin GPIO_PIN_0
#define Line_sensor15_GPIO_Port GPIOB
#define Line_sensor16_Pin GPIO_PIN_1
#define Line_sensor16_GPIO_Port GPIOB
#define Side_sensor1_Pin GPIO_PIN_2
#define Side_sensor1_GPIO_Port GPIOB
#define Side_sensor2_Pin GPIO_PIN_10
#define Side_sensor2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define M_Suction_PWM_Pin GPIO_PIN_6
#define M_Suction_PWM_GPIO_Port GPIOC
#define M_L_PWM_Pin GPIO_PIN_7
#define M_L_PWM_GPIO_Port GPIOC
#define M_L_PH_Pin GPIO_PIN_8
#define M_L_PH_GPIO_Port GPIOC
#define M_R_PWM_Pin GPIO_PIN_9
#define M_R_PWM_GPIO_Port GPIOC
#define M_R_PH_Pin GPIO_PIN_8
#define M_R_PH_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_12
#define LED5_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_15
#define SW2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOC
#define FLED3_Pin GPIO_PIN_11
#define FLED3_GPIO_Port GPIOC
#define FLED2_Pin GPIO_PIN_12
#define FLED2_GPIO_Port GPIOC
#define FLED1_Pin GPIO_PIN_2
#define FLED1_GPIO_Port GPIOD
#define ENC_L2_Pin GPIO_PIN_4
#define ENC_L2_GPIO_Port GPIOB
#define ENC_L1_Pin GPIO_PIN_5
#define ENC_L1_GPIO_Port GPIOB
#define ENC_R2_Pin GPIO_PIN_6
#define ENC_R2_GPIO_Port GPIOB
#define ENC_R1_Pin GPIO_PIN_7
#define ENC_R1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
