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
TIM_HandleTypeDef htim4;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAN_1_EFUSE_EN_Pin GPIO_PIN_2
#define FAN_1_EFUSE_EN_GPIO_Port GPIOE
#define STARTER_BUTTON_Pin GPIO_PIN_3
#define STARTER_BUTTON_GPIO_Port GPIOE
#define COOL_SWITCH_Pin GPIO_PIN_4
#define COOL_SWITCH_GPIO_Port GPIOE
#define COOL_SWITCH_EXTI_IRQn EXTI4_IRQn
#define MAX_COOL_SWITCH_Pin GPIO_PIN_5
#define MAX_COOL_SWITCH_GPIO_Port GPIOE
#define FAN_1_DLY_0_Pin GPIO_PIN_6
#define FAN_1_DLY_0_GPIO_Port GPIOE
#define FAN_1_DLY_1_Pin GPIO_PIN_13
#define FAN_1_DLY_1_GPIO_Port GPIOC
#define FAN_1_SET_0_Pin GPIO_PIN_14
#define FAN_1_SET_0_GPIO_Port GPIOC
#define FAN_1_SET_1_Pin GPIO_PIN_15
#define FAN_1_SET_1_GPIO_Port GPIOC
#define DASH_BUTTON_EFUSE_EN_Pin GPIO_PIN_0
#define DASH_BUTTON_EFUSE_EN_GPIO_Port GPIOH
#define DASH_BUTTON_EFUSE_DIAG_Pin GPIO_PIN_1
#define DASH_BUTTON_EFUSE_DIAG_GPIO_Port GPIOH
#define MUX2_OUT_Pin GPIO_PIN_2
#define MUX2_OUT_GPIO_Port GPIOF
#define INJECTION_EFUSE_EN_Pin GPIO_PIN_3
#define INJECTION_EFUSE_EN_GPIO_Port GPIOF
#define INJECTION_DLY_0_Pin GPIO_PIN_4
#define INJECTION_DLY_0_GPIO_Port GPIOF
#define INJECTION_EFUSE_DIAG_Pin GPIO_PIN_5
#define INJECTION_EFUSE_DIAG_GPIO_Port GPIOF
#define INJECTION_DLY_1_Pin GPIO_PIN_7
#define INJECTION_DLY_1_GPIO_Port GPIOF
#define INJECTION_SET_0_Pin GPIO_PIN_8
#define INJECTION_SET_0_GPIO_Port GPIOF
#define INJECTION_SET_1_Pin GPIO_PIN_0
#define INJECTION_SET_1_GPIO_Port GPIOC
#define MUX3_OUT_Pin GPIO_PIN_3
#define MUX3_OUT_GPIO_Port GPIOC
#define MUX_S0_Pin GPIO_PIN_0
#define MUX_S0_GPIO_Port GPIOA
#define MUX_S1_Pin GPIO_PIN_1
#define MUX_S1_GPIO_Port GPIOA
#define MUX_S2_Pin GPIO_PIN_2
#define MUX_S2_GPIO_Port GPIOA
#define IGNITION_EFUSE_DIAG_Pin GPIO_PIN_6
#define IGNITION_EFUSE_DIAG_GPIO_Port GPIOA
#define IGNITION_EFUSE_EN_Pin GPIO_PIN_7
#define IGNITION_EFUSE_EN_GPIO_Port GPIOA
#define STARTER_RELAY_EFUSE_DIAG_Pin GPIO_PIN_4
#define STARTER_RELAY_EFUSE_DIAG_GPIO_Port GPIOC
#define ECU_EFUSE_DIAG_Pin GPIO_PIN_5
#define ECU_EFUSE_DIAG_GPIO_Port GPIOC
#define ECU_EFUSE_EN_Pin GPIO_PIN_0
#define ECU_EFUSE_EN_GPIO_Port GPIOB
#define DAQ_EFUSE_EN_Pin GPIO_PIN_1
#define DAQ_EFUSE_EN_GPIO_Port GPIOB
#define MUX1_OUT_Pin GPIO_PIN_2
#define MUX1_OUT_GPIO_Port GPIOB
#define DAQ_EFUSE_DIAG_Pin GPIO_PIN_11
#define DAQ_EFUSE_DIAG_GPIO_Port GPIOF
#define ECU_DAQ_DLY_0_Pin GPIO_PIN_12
#define ECU_DAQ_DLY_0_GPIO_Port GPIOF
#define ECU_DAQ_DLY_1_Pin GPIO_PIN_13
#define ECU_DAQ_DLY_1_GPIO_Port GPIOF
#define ECU_DAQ_SET_0_Pin GPIO_PIN_14
#define ECU_DAQ_SET_0_GPIO_Port GPIOF
#define ECU_DAQ_SET_1_Pin GPIO_PIN_15
#define ECU_DAQ_SET_1_GPIO_Port GPIOF
#define IGNITION_DLY_0_Pin GPIO_PIN_0
#define IGNITION_DLY_0_GPIO_Port GPIOG
#define SHUTDOWN_SWITCH_MCU_Pin GPIO_PIN_1
#define SHUTDOWN_SWITCH_MCU_GPIO_Port GPIOG
#define IGNITION_DLY_1_Pin GPIO_PIN_7
#define IGNITION_DLY_1_GPIO_Port GPIOE
#define IGNITION_SET_0_Pin GPIO_PIN_8
#define IGNITION_SET_0_GPIO_Port GPIOE
#define IGNITION_SET_1_Pin GPIO_PIN_9
#define IGNITION_SET_1_GPIO_Port GPIOE
#define STARTER_RELAY_EFUSE_EN_Pin GPIO_PIN_10
#define STARTER_RELAY_EFUSE_EN_GPIO_Port GPIOE
#define FUEL_PUMP_DIAG_SEL_1_Pin GPIO_PIN_11
#define FUEL_PUMP_DIAG_SEL_1_GPIO_Port GPIOE
#define FUEL_PUMP_DIAG_SEL_0_Pin GPIO_PIN_12
#define FUEL_PUMP_DIAG_SEL_0_GPIO_Port GPIOE
#define FUEL_PUMP_FAULTRESET_Pin GPIO_PIN_13
#define FUEL_PUMP_FAULTRESET_GPIO_Port GPIOE
#define FUEL_PUMP_EN_Pin GPIO_PIN_14
#define FUEL_PUMP_EN_GPIO_Port GPIOE
#define SPARE_12V_DUAL_SET_MUX_1_Pin GPIO_PIN_15
#define SPARE_12V_DUAL_SET_MUX_1_GPIO_Port GPIOE
#define SPARE_12V_DUAL_SET_MUX_0_Pin GPIO_PIN_10
#define SPARE_12V_DUAL_SET_MUX_0_GPIO_Port GPIOB
#define SPARE_12V_DUAL_DLY_MUX_1_Pin GPIO_PIN_11
#define SPARE_12V_DUAL_DLY_MUX_1_GPIO_Port GPIOB
#define SPARE_12V_DUAL_DLY_MUX_0_Pin GPIO_PIN_12
#define SPARE_12V_DUAL_DLY_MUX_0_GPIO_Port GPIOB
#define SPARE_12V_3_DIAG_Pin GPIO_PIN_13
#define SPARE_12V_3_DIAG_GPIO_Port GPIOB
#define SPARE_12V_3_EN_Pin GPIO_PIN_14
#define SPARE_12V_3_EN_GPIO_Port GPIOB
#define RADIO_SET_1_Pin GPIO_PIN_15
#define RADIO_SET_1_GPIO_Port GPIOB
#define RADIO_DLY_0_Pin GPIO_PIN_8
#define RADIO_DLY_0_GPIO_Port GPIOD
#define SPARE_12V_2_DIAG_Pin GPIO_PIN_9
#define SPARE_12V_2_DIAG_GPIO_Port GPIOD
#define SPARE_12V_2_EN_Pin GPIO_PIN_10
#define SPARE_12V_2_EN_GPIO_Port GPIOD
#define RADIO_DLY_1_Pin GPIO_PIN_11
#define RADIO_DLY_1_GPIO_Port GPIOD
#define RADIO_SET_0_Pin GPIO_PIN_12
#define RADIO_SET_0_GPIO_Port GPIOD
#define RADIO_EFUSE_DIAG_Pin GPIO_PIN_13
#define RADIO_EFUSE_DIAG_GPIO_Port GPIOD
#define RADIO_EFUSE_EN_Pin GPIO_PIN_14
#define RADIO_EFUSE_EN_GPIO_Port GPIOD
#define ENGINE_SENSOR_DIAG_Pin GPIO_PIN_15
#define ENGINE_SENSOR_DIAG_GPIO_Port GPIOD
#define TELEMETRY_DISPLAY_DIAG_Pin GPIO_PIN_2
#define TELEMETRY_DISPLAY_DIAG_GPIO_Port GPIOG
#define TELEMETRY_DISPLAY_EN_Pin GPIO_PIN_3
#define TELEMETRY_DISPLAY_EN_GPIO_Port GPIOG
#define ENGINE_SENSOR_EN_Pin GPIO_PIN_4
#define ENGINE_SENSOR_EN_GPIO_Port GPIOG
#define TIRE_TEMP_PRESS_EN_Pin GPIO_PIN_5
#define TIRE_TEMP_PRESS_EN_GPIO_Port GPIOG
#define TIRE_TEMP_PRESS_DIAG_Pin GPIO_PIN_6
#define TIRE_TEMP_PRESS_DIAG_GPIO_Port GPIOG
#define POTENTIOMETER_EN_Pin GPIO_PIN_7
#define POTENTIOMETER_EN_GPIO_Port GPIOG
#define POTENTIOMETER_DIAG_Pin GPIO_PIN_8
#define POTENTIOMETER_DIAG_GPIO_Port GPIOG
#define BRAKE_PRESSURE_DIAG_Pin GPIO_PIN_6
#define BRAKE_PRESSURE_DIAG_GPIO_Port GPIOC
#define SPARE_5V_1_DIAG_Pin GPIO_PIN_7
#define SPARE_5V_1_DIAG_GPIO_Port GPIOC
#define SPARE_5V_1_EN_Pin GPIO_PIN_8
#define SPARE_5V_1_EN_GPIO_Port GPIOC
#define BRAKE_PRESSURE_EN_Pin GPIO_PIN_9
#define BRAKE_PRESSURE_EN_GPIO_Port GPIOC
#define BRAKE_LIGHT_EN_Pin GPIO_PIN_8
#define BRAKE_LIGHT_EN_GPIO_Port GPIOA
#define BRAKE_LIGHT_DIAG_Pin GPIO_PIN_9
#define BRAKE_LIGHT_DIAG_GPIO_Port GPIOA
#define SPARE_5V_2_EN_Pin GPIO_PIN_10
#define SPARE_5V_2_EN_GPIO_Port GPIOA
#define SPARE_5V_2_DIAG_Pin GPIO_PIN_2
#define SPARE_5V_2_DIAG_GPIO_Port GPIOH
#define FAN_2_EFUSE_EN_Pin GPIO_PIN_15
#define FAN_2_EFUSE_EN_GPIO_Port GPIOA
#define FAN_2_DLY_0_Pin GPIO_PIN_10
#define FAN_2_DLY_0_GPIO_Port GPIOC
#define FAN_2_DLY_1_Pin GPIO_PIN_11
#define FAN_2_DLY_1_GPIO_Port GPIOC
#define FAN_2_SET_0_Pin GPIO_PIN_12
#define FAN_2_SET_0_GPIO_Port GPIOC
#define FAN_2_SET_1_Pin GPIO_PIN_0
#define FAN_2_SET_1_GPIO_Port GPIOD
#define WATER_PUMP_DLY_0_Pin GPIO_PIN_1
#define WATER_PUMP_DLY_0_GPIO_Port GPIOD
#define WATER_PUMP_DLY_1_Pin GPIO_PIN_2
#define WATER_PUMP_DLY_1_GPIO_Port GPIOD
#define WATER_PUMP_SET_0_Pin GPIO_PIN_3
#define WATER_PUMP_SET_0_GPIO_Port GPIOD
#define WATER_PUMP_1_EN_Pin GPIO_PIN_4
#define WATER_PUMP_1_EN_GPIO_Port GPIOD
#define WATER_PUMP_SET_1_Pin GPIO_PIN_5
#define WATER_PUMP_SET_1_GPIO_Port GPIOD
#define WATER_PUMP_2_EN_Pin GPIO_PIN_6
#define WATER_PUMP_2_EN_GPIO_Port GPIOD
#define O2_SENSOR_EFUSE_EN_Pin GPIO_PIN_7
#define O2_SENSOR_EFUSE_EN_GPIO_Port GPIOD
#define O2_SENSOR_EFUSE_DIAG_Pin GPIO_PIN_9
#define O2_SENSOR_EFUSE_DIAG_GPIO_Port GPIOG
#define NTWS_EFUSE_EN_Pin GPIO_PIN_10
#define NTWS_EFUSE_EN_GPIO_Port GPIOG
#define NTWS_EFUSE_DIAG_Pin GPIO_PIN_11
#define NTWS_EFUSE_DIAG_GPIO_Port GPIOG
#define NTWSO2_DLY_0_Pin GPIO_PIN_12
#define NTWSO2_DLY_0_GPIO_Port GPIOG
#define NTWSO2_DLY_1_Pin GPIO_PIN_13
#define NTWSO2_DLY_1_GPIO_Port GPIOG
#define SPARE_12V_1_DIAG_SEL_1_Pin GPIO_PIN_14
#define SPARE_12V_1_DIAG_SEL_1_GPIO_Port GPIOG
#define SPARE_12V_1_DIAG_SEL_0_Pin GPIO_PIN_15
#define SPARE_12V_1_DIAG_SEL_0_GPIO_Port GPIOG
#define SPARE_1_FAULTRESET_Pin GPIO_PIN_4
#define SPARE_1_FAULTRESET_GPIO_Port GPIOB
#define SPARE_1_EFUSE_EN_Pin GPIO_PIN_5
#define SPARE_1_EFUSE_EN_GPIO_Port GPIOB
#define NTWSO2_SET_0_Pin GPIO_PIN_6
#define NTWSO2_SET_0_GPIO_Port GPIOB
#define NTWSO2_SET_1_Pin GPIO_PIN_7
#define NTWSO2_SET_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ADC_VREF 3.3
#define ADC_RES 4096
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
