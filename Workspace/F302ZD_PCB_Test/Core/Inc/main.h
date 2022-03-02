/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define MuxData_Pin GPIO_PIN_6
#define MuxData_GPIO_Port GPIOA
#define Mux_EN_Pin GPIO_PIN_7
#define Mux_EN_GPIO_Port GPIOA
#define GPIO_4_Pin GPIO_PIN_9
#define GPIO_4_GPIO_Port GPIOD
#define GPIO_3_Pin GPIO_PIN_10
#define GPIO_3_GPIO_Port GPIOD
#define GPIO_2_Pin GPIO_PIN_11
#define GPIO_2_GPIO_Port GPIOD
#define TIM4_PWM_GEN_Pin GPIO_PIN_12
#define TIM4_PWM_GEN_GPIO_Port GPIOD
#define S0_Pin GPIO_PIN_8
#define S0_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_9
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
