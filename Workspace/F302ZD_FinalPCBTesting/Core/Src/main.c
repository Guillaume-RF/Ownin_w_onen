/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HighPoweredFuse.h"
#include "PeripheralUtilities.h"
#include "Fuse12VSettings.h"
#include "Fuse12VPWM.h"
#include "LowPowerFuse.h"
#include "RadioFuse.h"
#include "VoltageSense.h"
#include "CurrentSense.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Number_HPFuses sizeof (HighPoweredFuses) / sizeof (*HighPoweredFuses)
#define Number_12VFuses sizeof(Fuses12V) / sizeof (*Fuses12V)
#define Number_12VPWMFuses sizeof (PWMedFuses) / sizeof (*PWMedFuses)
#define Number_LPFuses sizeof (LowPowerFuses) / sizeof (*LowPowerFuses)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for startupTask */
osThreadId_t startupTaskHandle;
const osThreadAttr_t startupTask_attributes = {
  .name = "startupTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for samplingTask */
osThreadId_t samplingTaskHandle;
const osThreadAttr_t samplingTask_attributes = {
  .name = "samplingTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for diagnosticCheckTask */
osThreadId_t diagnosticCheckTaskHandle;
const osThreadAttr_t diagnosticCheckTask_attributes = {
  .name = "diagnosticCheckTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for kickDogTask */
osThreadId_t kickDogTaskHandle;
const osThreadAttr_t kickDogTask_attributes = {
  .name = "kickDogTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for adc2Mutex */
osMutexId_t adc2MutexHandle;
const osMutexAttr_t adc2Mutex_attributes = {
  .name = "adc2Mutex"
};
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader_Fault;
CAN_TxHeaderTypeDef TxHeader_CriticalFault;
CAN_TxHeaderTypeDef TxHeader_FuseCurrent;
CAN_TxHeaderTypeDef TxHeader_RegCurrent;
CAN_TxHeaderTypeDef TxHeader_BatVoltage;

CAN_RxHeaderTypeDef RxHeader; //rx header

uint32_t canMailbox; //mailbox holder

uint8_t TxData_Faults[8];
uint8_t TxData_Sense[8];
uint8_t RxData[8];

uint8_t count = 0;

Fuse12VSettings Fan1Settings;
Fuse12VSettings Fan2Settings;
Fuse12VSettings ECUDAQSettings;
Fuse12VSettings WaterPumpsSettings;
Fuse12VSettings NTWSO2Settings;
Fuse12VSettings InjectionDashSettings;
Fuse12VSettings StarterIgnitionSettings;
Fuse12VSettings Spare12VSettings;
Fuse12VSettings RadioSettings;

HighPoweredFuse FuelPump;
HighPoweredFuse SpareHP;

Fuse12V_PWM Fan1;
Fuse12V_PWM Fan2;
Fuse12V_PWM WaterPump1;
Fuse12V_PWM WaterPump2;

Fuse12V ECU;
Fuse12V DAQ;
Fuse12V O2;
Fuse12V NTWS;
Fuse12V Dash;
Fuse12V Injection;
Fuse12V Ignition;
Fuse12V Starter;
Fuse12V Spare12V1;
Fuse12V Spare12V2;

RadioFuse Radio;

LowPowerFuse TelemetryDisplay;
LowPowerFuse TireTempPress;
LowPowerFuse BrakePressure;
LowPowerFuse BrakeLight;
LowPowerFuse EngineSensor;
LowPowerFuse Potentiometer;
LowPowerFuse Spare5V1;
LowPowerFuse Spare5V2;

VoltageSense VSense12V;

CurrentSense CSense5V;
CurrentSense CSense8V;

HighPoweredFuse *HighPoweredFuses[2] = {&FuelPump, &SpareHP};
Fuse12V_PWM *PWMedFuses[4] = {&Fan1, &Fan2, &WaterPump1, &WaterPump2};
Fuse12V *Fuses12V[10] = {&ECU, &DAQ, &O2, &NTWS, &Dash, &Injection, &Ignition, &Starter, &Spare12V1, &Spare12V2};
LowPowerFuse *LowPowerFuses[8] = {&TelemetryDisplay, &TireTempPress, &BrakePressure, &BrakeLight, &EngineSensor, &Potentiometer,
		&Spare5V1, &Spare5V2};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);
void StartStartupTask(void *argument);
void StartSamplingTask(void *argument);
void StartDiagnosticCheckTask(void *argument);
void StartKickDogTask(void *argument);

/* USER CODE BEGIN PFP */
void ConfigureMembers(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++)
	ITM_SendChar((*ptr++));
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //TIM4_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  ConfigureMembers();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //TODO: See if you can active notification on FIFO1 as well.

  TxHeader_Fault.DLC = 8;
  TxHeader_Fault.ExtId = 0;
  TxHeader_Fault.IDE = CAN_ID_STD;
  TxHeader_Fault.RTR = CAN_RTR_DATA;
  TxHeader_Fault.StdId = 0x103;
  TxHeader_Fault.TransmitGlobalTime = DISABLE;

  TxHeader_CriticalFault.DLC = 8;
  TxHeader_CriticalFault.ExtId = 0;
  TxHeader_CriticalFault.IDE = CAN_ID_STD;
  TxHeader_CriticalFault.RTR = CAN_RTR_DATA;
  TxHeader_CriticalFault.StdId = 0x104;
  TxHeader_CriticalFault.TransmitGlobalTime = DISABLE;

  TxHeader_FuseCurrent.DLC = 8;
  TxHeader_FuseCurrent.ExtId = 0;
  TxHeader_FuseCurrent.IDE = CAN_ID_STD;
  TxHeader_FuseCurrent.RTR = CAN_RTR_DATA;
  TxHeader_FuseCurrent.StdId = 0x105;
  TxHeader_FuseCurrent.TransmitGlobalTime = DISABLE;

  TxHeader_RegCurrent.DLC = 8;
  TxHeader_RegCurrent.ExtId = 0;
  TxHeader_RegCurrent.IDE = CAN_ID_STD;
  TxHeader_RegCurrent.RTR = CAN_RTR_DATA;
  TxHeader_RegCurrent.StdId = 0x106;
  TxHeader_RegCurrent.TransmitGlobalTime = DISABLE;

  TxHeader_BatVoltage.DLC = 8;
  TxHeader_BatVoltage.ExtId = 0;
  TxHeader_BatVoltage.IDE = CAN_ID_STD;
  TxHeader_BatVoltage.RTR = CAN_RTR_DATA;
  TxHeader_BatVoltage.StdId = 0x107;
  TxHeader_BatVoltage.TransmitGlobalTime = DISABLE;
	//Freeze watchdog when debugging.
	__HAL_DBGMCU_FREEZE_IWDG();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of adc2Mutex */
  adc2MutexHandle = osMutexNew(&adc2Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startupTask */
  startupTaskHandle = osThreadNew(StartStartupTask, NULL, &startupTask_attributes);

  /* creation of samplingTask */
  samplingTaskHandle = osThreadNew(StartSamplingTask, NULL, &samplingTask_attributes);

  /* creation of diagnosticCheckTask */
  diagnosticCheckTaskHandle = osThreadNew(StartDiagnosticCheckTask, NULL, &diagnosticCheckTask_attributes);

  /* creation of kickDogTask */
  kickDogTaskHandle = osThreadNew(StartKickDogTask, NULL, &kickDogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV16;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 50;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 500;
  hiwdg.Init.Reload = 2500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FAN_1_DLY_0_Pin|IGNITION_DLY_1_Pin|IGNITION_SET_0_Pin|IGNITION_SET_1_Pin
                          |STARTER_RELAY_EFUSE_EN_Pin|FUEL_PUMP_DIAG_SEL_1_Pin|FUEL_PUMP_DIAG_SEL_0_Pin|FUEL_PUMP_FAULTRESET_Pin
                          |FUEL_PUMP_EN_Pin|SPARE_12V_DUAL_SET_MUX_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FAN_1_DLY_1_Pin|FAN_1_SET_0_Pin|FAN_1_SET_1_Pin|INJECTION_SET_1_Pin
                          |FAN_2_DLY_0_Pin|FAN_2_DLY_1_Pin|FAN_2_SET_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DASH_BUTTON_EFUSE_EN_GPIO_Port, DASH_BUTTON_EFUSE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, INJECTION_EFUSE_EN_Pin|INJECTION_DLY_0_Pin|INJECTION_DLY_1_Pin|INJECTION_SET_0_Pin
                          |ECU_DAQ_DLY_0_Pin|ECU_DAQ_DLY_1_Pin|ECU_DAQ_SET_0_Pin|ECU_DAQ_SET_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX_S0_Pin|MUX_S1_Pin|MUX_S2_Pin|IGNITION_EFUSE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ECU_EFUSE_EN_Pin|DAQ_EFUSE_EN_Pin|SPARE_12V_DUAL_SET_MUX_0_Pin|SPARE_12V_DUAL_DLY_MUX_1_Pin
                          |SPARE_12V_DUAL_DLY_MUX_0_Pin|SPARE_12V_3_EN_Pin|RADIO_SET_1_Pin|SPARE_1_FAULTRESET_Pin
                          |SPARE_1_EFUSE_EN_Pin|NTWSO2_SET_0_Pin|NTWSO2_SET_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, IGNITION_DLY_0_Pin|NTWS_EFUSE_EN_Pin|NTWSO2_DLY_0_Pin|NTWSO2_DLY_1_Pin
                          |SPARE_12V_1_DIAG_SEL_1_Pin|SPARE_12V_1_DIAG_SEL_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RADIO_DLY_0_Pin|SPARE_12V_2_EN_Pin|RADIO_DLY_1_Pin|RADIO_SET_0_Pin
                          |RADIO_EFUSE_EN_Pin|FAN_2_SET_1_Pin|WATER_PUMP_DLY_0_Pin|WATER_PUMP_DLY_1_Pin
                          |WATER_PUMP_SET_0_Pin|WATER_PUMP_SET_1_Pin|O2_SENSOR_EFUSE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, TELEMETRY_DISPLAY_EN_Pin|ENGINE_SENSOR_EN_Pin|TIRE_TEMP_PRESS_EN_Pin|POTENTIOMETER_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPARE_5V_1_EN_Pin|BRAKE_PRESSURE_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BRAKE_LIGHT_EN_Pin|SPARE_5V_2_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : STARTER_BUTTON_Pin MAX_COOL_SWITCH_Pin */
  GPIO_InitStruct.Pin = STARTER_BUTTON_Pin|MAX_COOL_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : COOL_SWITCH_Pin */
  GPIO_InitStruct.Pin = COOL_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COOL_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_1_DLY_0_Pin IGNITION_DLY_1_Pin IGNITION_SET_0_Pin IGNITION_SET_1_Pin
                           STARTER_RELAY_EFUSE_EN_Pin FUEL_PUMP_DIAG_SEL_1_Pin FUEL_PUMP_DIAG_SEL_0_Pin FUEL_PUMP_FAULTRESET_Pin
                           FUEL_PUMP_EN_Pin SPARE_12V_DUAL_SET_MUX_1_Pin */
  GPIO_InitStruct.Pin = FAN_1_DLY_0_Pin|IGNITION_DLY_1_Pin|IGNITION_SET_0_Pin|IGNITION_SET_1_Pin
                          |STARTER_RELAY_EFUSE_EN_Pin|FUEL_PUMP_DIAG_SEL_1_Pin|FUEL_PUMP_DIAG_SEL_0_Pin|FUEL_PUMP_FAULTRESET_Pin
                          |FUEL_PUMP_EN_Pin|SPARE_12V_DUAL_SET_MUX_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_1_DLY_1_Pin FAN_1_SET_0_Pin FAN_1_SET_1_Pin INJECTION_SET_1_Pin
                           SPARE_5V_1_EN_Pin BRAKE_PRESSURE_EN_Pin FAN_2_DLY_0_Pin FAN_2_DLY_1_Pin
                           FAN_2_SET_0_Pin */
  GPIO_InitStruct.Pin = FAN_1_DLY_1_Pin|FAN_1_SET_0_Pin|FAN_1_SET_1_Pin|INJECTION_SET_1_Pin
                          |SPARE_5V_1_EN_Pin|BRAKE_PRESSURE_EN_Pin|FAN_2_DLY_0_Pin|FAN_2_DLY_1_Pin
                          |FAN_2_SET_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DASH_BUTTON_EFUSE_EN_Pin */
  GPIO_InitStruct.Pin = DASH_BUTTON_EFUSE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DASH_BUTTON_EFUSE_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DASH_BUTTON_EFUSE_DIAG_Pin SPARE_5V_2_DIAG_Pin */
  GPIO_InitStruct.Pin = DASH_BUTTON_EFUSE_DIAG_Pin|SPARE_5V_2_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX2_OUT_Pin */
  GPIO_InitStruct.Pin = MUX2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX2_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INJECTION_EFUSE_EN_Pin INJECTION_DLY_0_Pin INJECTION_DLY_1_Pin INJECTION_SET_0_Pin
                           ECU_DAQ_DLY_0_Pin ECU_DAQ_DLY_1_Pin ECU_DAQ_SET_0_Pin ECU_DAQ_SET_1_Pin */
  GPIO_InitStruct.Pin = INJECTION_EFUSE_EN_Pin|INJECTION_DLY_0_Pin|INJECTION_DLY_1_Pin|INJECTION_SET_0_Pin
                          |ECU_DAQ_DLY_0_Pin|ECU_DAQ_DLY_1_Pin|ECU_DAQ_SET_0_Pin|ECU_DAQ_SET_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : INJECTION_EFUSE_DIAG_Pin DAQ_EFUSE_DIAG_Pin */
  GPIO_InitStruct.Pin = INJECTION_EFUSE_DIAG_Pin|DAQ_EFUSE_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX3_OUT_Pin */
  GPIO_InitStruct.Pin = MUX3_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX3_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_S0_Pin MUX_S1_Pin MUX_S2_Pin IGNITION_EFUSE_EN_Pin
                           BRAKE_LIGHT_EN_Pin SPARE_5V_2_EN_Pin */
  GPIO_InitStruct.Pin = MUX_S0_Pin|MUX_S1_Pin|MUX_S2_Pin|IGNITION_EFUSE_EN_Pin
                          |BRAKE_LIGHT_EN_Pin|SPARE_5V_2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IGNITION_EFUSE_DIAG_Pin BRAKE_LIGHT_DIAG_Pin */
  GPIO_InitStruct.Pin = IGNITION_EFUSE_DIAG_Pin|BRAKE_LIGHT_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STARTER_RELAY_EFUSE_DIAG_Pin ECU_EFUSE_DIAG_Pin BRAKE_PRESSURE_DIAG_Pin SPARE_5V_1_DIAG_Pin */
  GPIO_InitStruct.Pin = STARTER_RELAY_EFUSE_DIAG_Pin|ECU_EFUSE_DIAG_Pin|BRAKE_PRESSURE_DIAG_Pin|SPARE_5V_1_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ECU_EFUSE_EN_Pin DAQ_EFUSE_EN_Pin SPARE_12V_DUAL_SET_MUX_0_Pin SPARE_12V_DUAL_DLY_MUX_1_Pin
                           SPARE_12V_DUAL_DLY_MUX_0_Pin SPARE_12V_3_EN_Pin RADIO_SET_1_Pin SPARE_1_FAULTRESET_Pin
                           SPARE_1_EFUSE_EN_Pin NTWSO2_SET_0_Pin NTWSO2_SET_1_Pin */
  GPIO_InitStruct.Pin = ECU_EFUSE_EN_Pin|DAQ_EFUSE_EN_Pin|SPARE_12V_DUAL_SET_MUX_0_Pin|SPARE_12V_DUAL_DLY_MUX_1_Pin
                          |SPARE_12V_DUAL_DLY_MUX_0_Pin|SPARE_12V_3_EN_Pin|RADIO_SET_1_Pin|SPARE_1_FAULTRESET_Pin
                          |SPARE_1_EFUSE_EN_Pin|NTWSO2_SET_0_Pin|NTWSO2_SET_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IGNITION_DLY_0_Pin TELEMETRY_DISPLAY_EN_Pin ENGINE_SENSOR_EN_Pin TIRE_TEMP_PRESS_EN_Pin
                           POTENTIOMETER_EN_Pin NTWS_EFUSE_EN_Pin NTWSO2_DLY_0_Pin NTWSO2_DLY_1_Pin
                           SPARE_12V_1_DIAG_SEL_1_Pin SPARE_12V_1_DIAG_SEL_0_Pin */
  GPIO_InitStruct.Pin = IGNITION_DLY_0_Pin|TELEMETRY_DISPLAY_EN_Pin|ENGINE_SENSOR_EN_Pin|TIRE_TEMP_PRESS_EN_Pin
                          |POTENTIOMETER_EN_Pin|NTWS_EFUSE_EN_Pin|NTWSO2_DLY_0_Pin|NTWSO2_DLY_1_Pin
                          |SPARE_12V_1_DIAG_SEL_1_Pin|SPARE_12V_1_DIAG_SEL_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUTDOWN_SWITCH_MCU_Pin TELEMETRY_DISPLAY_DIAG_Pin TIRE_TEMP_PRESS_DIAG_Pin POTENTIOMETER_DIAG_Pin
                           O2_SENSOR_EFUSE_DIAG_Pin NTWS_EFUSE_DIAG_Pin */
  GPIO_InitStruct.Pin = SHUTDOWN_SWITCH_MCU_Pin|TELEMETRY_DISPLAY_DIAG_Pin|TIRE_TEMP_PRESS_DIAG_Pin|POTENTIOMETER_DIAG_Pin
                          |O2_SENSOR_EFUSE_DIAG_Pin|NTWS_EFUSE_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SPARE_12V_3_DIAG_Pin */
  GPIO_InitStruct.Pin = SPARE_12V_3_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPARE_12V_3_DIAG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_DLY_0_Pin SPARE_12V_2_EN_Pin RADIO_DLY_1_Pin RADIO_SET_0_Pin
                           RADIO_EFUSE_EN_Pin FAN_2_SET_1_Pin WATER_PUMP_DLY_0_Pin WATER_PUMP_DLY_1_Pin
                           WATER_PUMP_SET_0_Pin WATER_PUMP_SET_1_Pin O2_SENSOR_EFUSE_EN_Pin */
  GPIO_InitStruct.Pin = RADIO_DLY_0_Pin|SPARE_12V_2_EN_Pin|RADIO_DLY_1_Pin|RADIO_SET_0_Pin
                          |RADIO_EFUSE_EN_Pin|FAN_2_SET_1_Pin|WATER_PUMP_DLY_0_Pin|WATER_PUMP_DLY_1_Pin
                          |WATER_PUMP_SET_0_Pin|WATER_PUMP_SET_1_Pin|O2_SENSOR_EFUSE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPARE_12V_2_DIAG_Pin RADIO_EFUSE_DIAG_Pin ENGINE_SENSOR_DIAG_Pin */
  GPIO_InitStruct.Pin = SPARE_12V_2_DIAG_Pin|RADIO_EFUSE_DIAG_Pin|ENGINE_SENSOR_DIAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void ConfigureMembers(void)
{
	/*High Powered Fuses*/
	/*Fuel Pump*/
	FuelPump.port_input = FUEL_PUMP_EN_GPIO_Port;
	FuelPump.port_senseSelect0 = FUEL_PUMP_DIAG_SEL_0_GPIO_Port;
	FuelPump.port_senseSelect1 = FUEL_PUMP_DIAG_SEL_1_GPIO_Port;
	FuelPump.port_faultRST = FUEL_PUMP_FAULTRESET_GPIO_Port;

	FuelPump.pin_input = FUEL_PUMP_EN_Pin;
	FuelPump.pin_senseSelect0 = FUEL_PUMP_DIAG_SEL_0_Pin;
	FuelPump.pin_senseSelect1 = FUEL_PUMP_DIAG_SEL_1_Pin;
	FuelPump.pin_faultRST = FUEL_PUMP_FAULTRESET_Pin;

	FuelPump.ADC_multiSense = &hadc2;
	FuelPump.ADC_channel = ADC_CHANNEL_12;
	FuelPump.mux_channel = 1;
	FuelPump.currentGain = 3440;
	FuelPump.currentShunt = 649;

	FuelPump.ID = 0;
	FuelPump.time_ms_lastRetryProcedure = 0;
	FuelPump.criticalFault = 0;

	/*Spare high powered fuse*/
	SpareHP.port_input = SPARE_1_EFUSE_EN_GPIO_Port;
	SpareHP.port_senseSelect0 = SPARE_12V_1_DIAG_SEL_0_GPIO_Port;
	SpareHP.port_senseSelect1 = SPARE_12V_1_DIAG_SEL_1_GPIO_Port;
	SpareHP.port_faultRST = SPARE_1_FAULTRESET_GPIO_Port;

	SpareHP.pin_input = SPARE_1_EFUSE_EN_Pin;
	SpareHP.pin_senseSelect0 = SPARE_12V_1_DIAG_SEL_0_Pin;
	SpareHP.pin_senseSelect1 = SPARE_12V_1_DIAG_SEL_1_Pin;
	SpareHP.pin_faultRST = FUEL_PUMP_FAULTRESET_Pin;

	SpareHP.ADC_multiSense = &hadc2;
	SpareHP.ADC_channel = ADC_CHANNEL_9;
	SpareHP.mux_channel = 3;
	SpareHP.currentGain = 3440;
	SpareHP.currentShunt = 649;

	SpareHP.ID = 1;
	SpareHP.time_ms_lastRetryProcedure = 0;
	SpareHP.criticalFault = 0;

	/*12V fuse settings.*/
	/*Fan1 settings*/
	Fan1Settings.port_currentMux0 = FAN_1_SET_0_GPIO_Port;
	Fan1Settings.port_currentMux1 = FAN_1_SET_1_GPIO_Port;
	Fan1Settings.port_delayMux0 = FAN_1_DLY_0_GPIO_Port;
	Fan1Settings.port_delayMux1 = FAN_1_DLY_1_GPIO_Port;

	Fan1Settings.pin_currentMux0 = FAN_1_SET_0_Pin;
	Fan1Settings.pin_currentMux1 = FAN_1_SET_1_Pin;
	Fan1Settings.pin_delayMux0 = FAN_1_DLY_0_Pin;
	Fan1Settings.pin_delayMux1 = FAN_1_DLY_1_Pin;

	/*Fan2 settings*/
	Fan2Settings.port_currentMux0 = FAN_2_SET_0_GPIO_Port;
	Fan2Settings.port_currentMux1 = FAN_2_SET_1_GPIO_Port;
	Fan2Settings.port_delayMux0 = FAN_2_DLY_0_GPIO_Port;
	Fan2Settings.port_delayMux1 = FAN_2_DLY_1_GPIO_Port;

	Fan2Settings.pin_currentMux0 = FAN_2_SET_0_Pin;
	Fan2Settings.pin_currentMux1 = FAN_2_SET_1_Pin;
	Fan2Settings.pin_delayMux0 = FAN_2_DLY_0_Pin;
	Fan2Settings.pin_delayMux1 = FAN_2_DLY_1_Pin;

	/*Water pumps settings*/
	ECUDAQSettings.port_currentMux0 = ECU_DAQ_SET_0_GPIO_Port;
	ECUDAQSettings.port_currentMux1 = ECU_DAQ_SET_1_GPIO_Port;
	ECUDAQSettings.port_delayMux0 = ECU_DAQ_DLY_0_GPIO_Port;
	ECUDAQSettings.port_delayMux1 = ECU_DAQ_DLY_1_GPIO_Port;

	ECUDAQSettings.pin_currentMux0 = ECU_DAQ_SET_0_Pin;
	ECUDAQSettings.pin_currentMux1 = ECU_DAQ_SET_1_Pin;
	ECUDAQSettings.pin_delayMux0 = ECU_DAQ_DLY_0_Pin;
	ECUDAQSettings.pin_delayMux1 = ECU_DAQ_DLY_1_Pin;

	/*ECU & DAQ settings*/
	WaterPumpsSettings.port_currentMux0 = WATER_PUMP_SET_0_GPIO_Port;
	WaterPumpsSettings.port_currentMux1 = WATER_PUMP_SET_1_GPIO_Port;
	WaterPumpsSettings.port_delayMux0 = WATER_PUMP_DLY_0_GPIO_Port;
	WaterPumpsSettings.port_delayMux1 = WATER_PUMP_DLY_1_GPIO_Port;

	WaterPumpsSettings.pin_currentMux0 = WATER_PUMP_SET_0_Pin;
	WaterPumpsSettings.pin_currentMux1 = WATER_PUMP_SET_1_Pin;
	WaterPumpsSettings.pin_delayMux0 = WATER_PUMP_DLY_0_Pin;
	WaterPumpsSettings.pin_delayMux1 = WATER_PUMP_DLY_1_Pin;

	/*NTWS & O2 Settings	 */
	NTWSO2Settings.port_currentMux0 = NTWSO2_SET_0_GPIO_Port;
	NTWSO2Settings.port_currentMux1 = NTWSO2_SET_1_GPIO_Port;
	NTWSO2Settings.port_delayMux0 = NTWSO2_DLY_0_GPIO_Port;
	NTWSO2Settings.port_delayMux1 = NTWSO2_DLY_1_GPIO_Port;

	NTWSO2Settings.pin_currentMux0 = NTWSO2_SET_0_Pin;
	NTWSO2Settings.pin_currentMux1 = NTWSO2_SET_1_Pin;
	NTWSO2Settings.pin_delayMux0 = NTWSO2_DLY_0_Pin;
	NTWSO2Settings.pin_delayMux1 = NTWSO2_DLY_1_Pin;

	/*Injection and dash settings */
	InjectionDashSettings.port_currentMux0 = INJECTION_SET_0_GPIO_Port;
	InjectionDashSettings.port_currentMux1 = INJECTION_SET_1_GPIO_Port;
	InjectionDashSettings.port_delayMux0 = INJECTION_DLY_0_GPIO_Port;
	InjectionDashSettings.port_delayMux1 = INJECTION_DLY_1_GPIO_Port;

	InjectionDashSettings.pin_currentMux0 = INJECTION_SET_0_Pin;
	InjectionDashSettings.pin_currentMux1 = INJECTION_SET_1_Pin;
	InjectionDashSettings.pin_delayMux0 = INJECTION_DLY_0_Pin;
	InjectionDashSettings.pin_delayMux1 = INJECTION_DLY_1_Pin;

	/*Starter and ignition settings*/
	StarterIgnitionSettings.port_currentMux0 = IGNITION_SET_0_GPIO_Port;
	StarterIgnitionSettings.port_currentMux1 = IGNITION_SET_1_GPIO_Port;
	StarterIgnitionSettings.port_delayMux0 = IGNITION_DLY_0_GPIO_Port;
	StarterIgnitionSettings.port_delayMux1 = IGNITION_DLY_1_GPIO_Port;

	StarterIgnitionSettings.pin_currentMux0 = IGNITION_SET_0_Pin;
	StarterIgnitionSettings.pin_currentMux1 = IGNITION_SET_1_Pin;
	StarterIgnitionSettings.pin_delayMux0 = IGNITION_DLY_0_Pin;
	StarterIgnitionSettings.pin_delayMux1 = IGNITION_DLY_0_Pin;

	/*Spare 12V setting*/
	Spare12VSettings.port_currentMux0 = SPARE_12V_DUAL_SET_MUX_0_GPIO_Port;
	Spare12VSettings.port_currentMux1 = SPARE_12V_DUAL_SET_MUX_1_GPIO_Port;
	Spare12VSettings.port_delayMux0 = SPARE_12V_DUAL_DLY_MUX_0_GPIO_Port;
	Spare12VSettings.port_delayMux1 = SPARE_12V_DUAL_DLY_MUX_1_GPIO_Port;

	Spare12VSettings.pin_currentMux0 = SPARE_12V_DUAL_SET_MUX_0_Pin;
	Spare12VSettings.pin_currentMux1 = SPARE_12V_DUAL_SET_MUX_1_Pin;
	Spare12VSettings.pin_delayMux0 = SPARE_12V_DUAL_DLY_MUX_0_Pin;
	Spare12VSettings.pin_delayMux1 = SPARE_12V_DUAL_DLY_MUX_1_Pin;

	/*Radio settings*/
	RadioSettings.port_currentMux0 = RADIO_SET_0_GPIO_Port;
	RadioSettings.port_currentMux1 = RADIO_SET_1_GPIO_Port;
	RadioSettings.port_delayMux0 = RADIO_DLY_0_GPIO_Port;
	RadioSettings.port_delayMux1 = RADIO_DLY_1_GPIO_Port;

	RadioSettings.pin_currentMux0 = RADIO_SET_0_Pin;
	RadioSettings.pin_currentMux1 = RADIO_SET_1_Pin;
	RadioSettings.pin_delayMux0 = RADIO_DLY_0_Pin;
	RadioSettings.pin_delayMux1 = RADIO_DLY_1_Pin;

	/*12V PWM'ed fuses.*/
	/*Fan 1*/
	Fan1.TIM_input = &htim3;
	Fan1.TIM_channel = TIM_CHANNEL_1;
	Fan1.dutyCycle = 0;
	Fan1.frequency = 1000;

	Fan1.ADC_diagnostic = &hadc2;
	Fan1.ADC_diagnosticChannel = ADC_CHANNEL_9;
	Fan1.mux_diagnosticChannel = 5;

	Fan1.ADC_currentSense = &hadc2;
	Fan1.ADC_currentSenseChannel = ADC_CHANNEL_12;
	Fan1.mux_currentSenseChannel = 3;

	Fan1.currentGain = 20;
	Fan1.currentShunt = 0.018;

	Fan1.ID = 2;
	Fan1.time_ms_lastRetryProcedure = 0;
	Fan1.criticalFault = 0;

	Fan1.settings = &Fan1Settings;

	/*Fan 2*/
	Fan2.TIM_input = &htim2;
	Fan2.TIM_channel = TIM_CHANNEL_1;
	Fan2.dutyCycle = 0;
	Fan2.frequency = 1000;

	Fan2.ADC_diagnostic = &hadc2;
	Fan2.ADC_diagnosticChannel = ADC_CHANNEL_10;
	Fan2.mux_diagnosticChannel = 5;

	Fan2.ADC_currentSense = &hadc2;
	Fan2.ADC_currentSenseChannel = ADC_CHANNEL_9;
	Fan2.mux_currentSenseChannel = 4;

	Fan2.currentGain = 20;
	Fan2.currentShunt = 0.018;

	Fan2.ID = 3;
	Fan2.time_ms_lastRetryProcedure = 0;
	Fan2.criticalFault = 0;

	Fan2.settings = &Fan2Settings;

	/*Water pump 1*/
	WaterPump1.TIM_input = &htim2;
	WaterPump1.TIM_channel = TIM_CHANNEL_2;
	WaterPump1.dutyCycle = 0;
	WaterPump1.frequency = 1000;

	WaterPump1.ADC_diagnostic = &hadc2;
	WaterPump1.ADC_diagnosticChannel = ADC_CHANNEL_10;
	WaterPump1.mux_diagnosticChannel = 6;

	WaterPump1.ADC_currentSense = &hadc2;
	WaterPump1.ADC_currentSenseChannel = ADC_CHANNEL_12;
	WaterPump1.mux_currentSenseChannel = 3;

	WaterPump1.currentGain = 20;
	WaterPump1.currentShunt = 0.018;

	WaterPump1.ID = 4;
	WaterPump1.time_ms_lastRetryProcedure = 0;
	WaterPump1.criticalFault = 0;

	WaterPump1.settings = &WaterPumpsSettings;

	/*Water pump 2*/
	WaterPump2.TIM_input = &htim2;
	WaterPump2.TIM_channel = TIM_CHANNEL_4;
	WaterPump2.dutyCycle = 0;
	WaterPump2.frequency = 1000;

	WaterPump2.ADC_diagnostic = &hadc2;
	WaterPump2.ADC_diagnosticChannel = ADC_CHANNEL_10;
	WaterPump2.mux_diagnosticChannel = 7;

	WaterPump2.ADC_currentSense = &hadc2;
	WaterPump2.ADC_currentSenseChannel = ADC_CHANNEL_9;
	WaterPump2.mux_currentSenseChannel = 2;

	WaterPump2.currentGain = 20;
	WaterPump2.currentShunt = 0.018;

	WaterPump2.ID = 5;
	WaterPump2.time_ms_lastRetryProcedure = 0;
	WaterPump2.criticalFault = 0;

	WaterPump2.settings = &WaterPumpsSettings;

	/*12V fuses*/
	/*ECU*/
	ECU.port_input = ECU_EFUSE_EN_GPIO_Port;
	ECU.port_diagnostic = ECU_EFUSE_DIAG_GPIO_Port;

	ECU.pin_input = ECU_EFUSE_EN_Pin;
	ECU.pin_diagnostic = ECU_EFUSE_DIAG_Pin;

	ECU.ADC_currentSense = &hadc2;
	ECU.ADC_channel = ADC_CHANNEL_12;
	ECU.mux_channel = 7;

	ECU.currentGain = 20;
	ECU.currentShunt = 0.165;

	ECU.ID = 6;
	ECU.time_ms_lastRetryProcedure = 0;
	ECU.criticalFault = 0;

	ECU.settings = &ECUDAQSettings;

	/*DAQ*/
	DAQ.port_input = DAQ_EFUSE_EN_GPIO_Port;
	DAQ.port_diagnostic = DAQ_EFUSE_DIAG_GPIO_Port;

	DAQ.pin_input = DAQ_EFUSE_EN_Pin;
	DAQ.pin_diagnostic = DAQ_EFUSE_DIAG_Pin;

	DAQ.ADC_currentSense = &hadc2;
	DAQ.ADC_channel = ADC_CHANNEL_12;
	DAQ.mux_channel = 6;

	DAQ.currentGain = 20;
	DAQ.currentShunt = 0.165;

	DAQ.ID = 7;
	DAQ.time_ms_lastRetryProcedure = 0;
	DAQ.criticalFault = 0;

	DAQ.settings = &ECUDAQSettings;

	/*O2*/
	O2.port_input = O2_SENSOR_EFUSE_EN_GPIO_Port;
	O2.port_diagnostic = O2_SENSOR_EFUSE_DIAG_GPIO_Port;

	O2.pin_input = O2_SENSOR_EFUSE_EN_Pin;
	O2.pin_diagnostic = O2_SENSOR_EFUSE_DIAG_Pin;

	O2.ADC_currentSense = &hadc2;
	O2.ADC_channel = ADC_CHANNEL_9;
	O2.mux_channel = 0;

	O2.currentGain = 20;
	O2.currentShunt = 0.165;

	O2.ID = 8;
	O2.time_ms_lastRetryProcedure = 0;
	O2.criticalFault = 0;

	O2.settings = &NTWSO2Settings;

	/*NTWS*/
	NTWS.port_input = NTWS_EFUSE_EN_GPIO_Port;
	NTWS.port_diagnostic = NTWS_EFUSE_DIAG_GPIO_Port;

	NTWS.pin_input = NTWS_EFUSE_EN_Pin;
	NTWS.pin_diagnostic = NTWS_EFUSE_DIAG_Pin;

	NTWS.ADC_currentSense = &hadc2;
	NTWS.ADC_channel = ADC_CHANNEL_9;
	NTWS.mux_channel = 1;

	NTWS.currentGain = 20;
	NTWS.currentShunt = 0.165;

	NTWS.ID = 9;
	NTWS.time_ms_lastRetryProcedure = 0;
	NTWS.criticalFault = 0;

	NTWS.settings = &NTWSO2Settings;

	/*Dash*/
	Dash.port_input = DASH_BUTTON_EFUSE_EN_GPIO_Port;
	Dash.port_diagnostic = DASH_BUTTON_EFUSE_DIAG_GPIO_Port;

	Dash.pin_input = DASH_BUTTON_EFUSE_EN_Pin;
	Dash.pin_diagnostic = DASH_BUTTON_EFUSE_DIAG_Pin;

	Dash.ADC_currentSense = &hadc2;
	Dash.ADC_channel = ADC_CHANNEL_9;
	Dash.mux_channel = 6;

	Dash.currentGain = 20;
	Dash.currentShunt = 0.033;

	Dash.ID = 10;
	Dash.time_ms_lastRetryProcedure = 0;
	Dash.criticalFault = 0;

	Dash.settings = &InjectionDashSettings;

	/*Injection*/
	Injection.port_input = INJECTION_EFUSE_EN_GPIO_Port;
	Injection.port_diagnostic = INJECTION_EFUSE_DIAG_GPIO_Port;

	Injection.pin_input = INJECTION_EFUSE_EN_Pin;
	Injection.pin_diagnostic = INJECTION_EFUSE_DIAG_Pin;

	Injection.ADC_currentSense = &hadc2;
	Injection.ADC_channel = ADC_CHANNEL_9;
	Injection.mux_channel = 7;

	Injection.currentGain = 20;
	Injection.currentShunt = 0.033;

	Injection.ID = 11;
	Injection.time_ms_lastRetryProcedure = 0;
	Injection.criticalFault = 0;

	Injection.settings = &InjectionDashSettings;

	/*Ignition*/
	Ignition.port_input = IGNITION_EFUSE_EN_GPIO_Port;
	Ignition.port_diagnostic = IGNITION_EFUSE_DIAG_GPIO_Port;

	Ignition.pin_input = IGNITION_EFUSE_EN_Pin;
	Ignition.pin_diagnostic = IGNITION_EFUSE_DIAG_Pin;

	Ignition.ADC_currentSense = &hadc2;
	Ignition.ADC_channel = ADC_CHANNEL_12;
	Ignition.mux_channel = 4;

	Ignition.currentGain = 20;
	Ignition.currentShunt = 0.165;

	Ignition.ID = 12;
	Ignition.time_ms_lastRetryProcedure = 0;
	Ignition.criticalFault = 0;

	Ignition.settings = &StarterIgnitionSettings;

	/*Starter*/
	Starter.port_input = STARTER_RELAY_EFUSE_EN_GPIO_Port;
	Starter.port_diagnostic = STARTER_RELAY_EFUSE_DIAG_GPIO_Port;

	Starter.pin_input = STARTER_RELAY_EFUSE_EN_Pin;
	Starter.pin_diagnostic = STARTER_RELAY_EFUSE_DIAG_Pin;

	Starter.ADC_currentSense = &hadc2;
	Starter.ADC_channel = ADC_CHANNEL_12;
	Starter.mux_channel = 2;

	Starter.currentGain = 20;
	Starter.currentShunt = 0.165;

	Starter.ID = 13;
	Starter.time_ms_lastRetryProcedure = 0;
	Starter.criticalFault = 0;

	Starter.settings = &StarterIgnitionSettings;

	/*Spare12V1*/
	Spare12V1.port_input = SPARE_12V_2_EN_GPIO_Port;
	Spare12V1.port_diagnostic = SPARE_12V_2_DIAG_GPIO_Port;

	Spare12V1.pin_input = SPARE_12V_2_EN_Pin;
	Spare12V1.pin_diagnostic = SPARE_12V_2_DIAG_Pin;

	Spare12V1.ADC_currentSense = &hadc2;
	Spare12V1.ADC_channel = ADC_CHANNEL_10;
	Spare12V1.mux_channel = 0;

	Spare12V1.currentGain = 20;
	Spare12V1.currentShunt = 0.033;

	Spare12V1.ID = 14;
	Spare12V1.time_ms_lastRetryProcedure = 0;
	Spare12V1.criticalFault = 0;

	Spare12V1.settings = &Spare12VSettings;

	/*Spare12V2*/
	Spare12V2.port_input = SPARE_12V_3_EN_GPIO_Port;
	Spare12V2.port_diagnostic = SPARE_12V_3_DIAG_GPIO_Port;

	Spare12V2.pin_input = SPARE_12V_3_EN_Pin;
	Spare12V2.pin_diagnostic = SPARE_12V_3_DIAG_Pin;

	Spare12V2.ADC_currentSense = &hadc2;
	Spare12V2.ADC_channel = ADC_CHANNEL_10;
	Spare12V2.mux_channel = 3;

	Spare12V2.currentGain = 20;
	Spare12V2.currentShunt = 0.165;

	Spare12V2.ID = 15;
	Spare12V2.time_ms_lastRetryProcedure = 0;
	Spare12V2.criticalFault = 0;

	Spare12V2.settings = &Spare12VSettings;

	/*Radio fuses*/
	Radio.port_input = RADIO_EFUSE_EN_GPIO_Port;
	Radio.port_diagnostic = RADIO_EFUSE_DIAG_GPIO_Port;

	Radio.pin_input = RADIO_EFUSE_EN_Pin;
	Radio.pin_diagnostic = RADIO_EFUSE_DIAG_Pin;

	Radio.ID = 16;
	Radio.time_ms_lastRetryProcedure = 0;
	Radio.criticalFault = 0;

	Radio.settings = &RadioSettings;

	/*5V fuses*/
	/*Telemetry/Display*/
	TelemetryDisplay.port_input = TELEMETRY_DISPLAY_EN_GPIO_Port;
	TelemetryDisplay.port_diagnostic = TELEMETRY_DISPLAY_DIAG_GPIO_Port;

	TelemetryDisplay.pin_input = TELEMETRY_DISPLAY_EN_Pin;
	TelemetryDisplay.pin_diagnostic = TELEMETRY_DISPLAY_DIAG_Pin;

	TelemetryDisplay.ID = 17;
	TelemetryDisplay.time_ms_lastRetryProcedure = 0;
	TelemetryDisplay.criticalFault = 0;

	/*TireTempPress*/
	TireTempPress.port_input = TIRE_TEMP_PRESS_EN_GPIO_Port;
	TireTempPress.port_diagnostic = TIRE_TEMP_PRESS_DIAG_GPIO_Port;

	TireTempPress.pin_input = TIRE_TEMP_PRESS_EN_Pin;
	TireTempPress.pin_diagnostic = TIRE_TEMP_PRESS_DIAG_Pin;

	TireTempPress.ID = 18;
	TireTempPress.time_ms_lastRetryProcedure = 0;
	TireTempPress.criticalFault = 0;

	/*BrakePressure*/
	BrakePressure.port_input = BRAKE_PRESSURE_EN_GPIO_Port;
	BrakePressure.port_diagnostic = BRAKE_PRESSURE_DIAG_GPIO_Port;

	BrakePressure.pin_input = BRAKE_PRESSURE_EN_Pin;
	BrakePressure.pin_diagnostic = BRAKE_PRESSURE_DIAG_Pin;

	BrakePressure.ID = 19;
	BrakePressure.time_ms_lastRetryProcedure = 0;
	BrakePressure.criticalFault = 0;

	/*BrakeLight*/
	BrakeLight.port_input = BRAKE_LIGHT_EN_GPIO_Port;
	BrakeLight.port_diagnostic = BRAKE_LIGHT_DIAG_GPIO_Port;

	BrakeLight.pin_input = BRAKE_LIGHT_EN_Pin;
	BrakeLight.pin_diagnostic = BRAKE_LIGHT_DIAG_Pin;

	BrakeLight.ID = 20;
	BrakeLight.time_ms_lastRetryProcedure = 0;
	BrakeLight.criticalFault = 0;

	/*Engine Sensor*/
	EngineSensor.port_input = ENGINE_SENSOR_EN_GPIO_Port;
	EngineSensor.port_diagnostic = ENGINE_SENSOR_DIAG_GPIO_Port;

	EngineSensor.pin_input = ENGINE_SENSOR_EN_Pin;
	EngineSensor.pin_diagnostic = ENGINE_SENSOR_DIAG_Pin;

	EngineSensor.ID = 21;
	EngineSensor.time_ms_lastRetryProcedure = 0;
	EngineSensor.criticalFault = 0;

	/*Potentiometer*/
	Potentiometer.port_input = POTENTIOMETER_EN_GPIO_Port;
	Potentiometer.port_diagnostic = POTENTIOMETER_DIAG_GPIO_Port;

	Potentiometer.pin_input = POTENTIOMETER_EN_Pin;
	Potentiometer.pin_diagnostic = POTENTIOMETER_DIAG_Pin;

	Potentiometer.ID = 22;
	Potentiometer.time_ms_lastRetryProcedure = 0;
	Potentiometer.criticalFault = 0;

	/*Spare 5V 1*/
	Spare5V1.port_input = SPARE_5V_1_EN_GPIO_Port;
	Spare5V1.port_diagnostic = SPARE_5V_1_DIAG_GPIO_Port;

	Spare5V1.pin_input = SPARE_5V_1_EN_Pin;
	Spare5V1.pin_diagnostic = SPARE_5V_1_DIAG_Pin;

	Spare5V1.ID = 23;
	Spare5V1.time_ms_lastRetryProcedure = 0;
	Spare5V1.criticalFault = 0;

	/*Spare 5V 2*/
	Spare5V2.port_input = SPARE_5V_2_EN_GPIO_Port;
	Spare5V2.port_diagnostic = SPARE_5V_2_DIAG_GPIO_Port;

	Spare5V2.pin_input = SPARE_5V_2_EN_Pin;
	Spare5V2.pin_diagnostic = SPARE_5V_2_DIAG_Pin;

	Spare5V2.ID = 24;
	Spare5V2.time_ms_lastRetryProcedure = 0;
	Spare5V2.criticalFault = 0;

	/*Battery voltage sense*/
	VSense12V.ADC_voltageSense = &hadc2;
	VSense12V.ADC_channel = ADC_CHANNEL_10;
	VSense12V.mux_channel = 4;

	VSense12V.voltageDivision = 4.54;

	/*Current sense*/
	/*5V regulator*/
	CSense5V.ADC_currentSense = &hadc2;
	CSense5V.ADC_channel = ADC_CHANNEL_10;
	CSense5V.mux_channel = 1;

	CSense5V.currentGain = 20;
	CSense5V.currentShunt = 0.055;

	/*8V regulator*/
	CSense8V.ADC_currentSense = &hadc2;
	CSense8V.ADC_channel = ADC_CHANNEL_10;
	CSense8V.mux_channel = 2;

	CSense8V.currentGain = 20;
	CSense8V.currentShunt = 0.055;
}

void StartupSelfTest(void)
{

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	count++;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == COOL_SWITCH_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	float dutyCycle = Fan1.dutyCycle + 0.25;
    	if(dutyCycle > 1)
    	{
    		dutyCycle = 0;
    	}
    	Fuse12VPWM_SetInputDutyCycle(&Fan1, dutyCycle);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartStartupTask */
/**
  * @brief  Function implementing the startupTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStartupTask */
void StartStartupTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/*Group 0*/
	Fuse12VPWM_SetTripTime(&Fan1, ms_200);
	Fuse12VPWM_SetCurrentLimit(&Fan1, A_7);
	Fuse12VPWM_SetInputFrequency(&Fan1, 500);
	Fuse12VPWM_SetInputDutyCycle(&Fan1, 0.5);
	Fuse12VPWM_StartPWM(&Fan1);

	/*Group 1*/
	Fuse12VPWM_SetTripTime(&Fan2, ms_200);
	Fuse12VPWM_SetCurrentLimit(&Fan2, A_11_5);
	Fuse12VPWM_SetInputFrequency(&Fan2, 500);
	Fuse12VPWM_SetInputDutyCycle(&Fan2, 0.5);
	Fuse12VPWM_StartPWM(&Fan2);
	HAL_Delay(150);

	/*Group 2*/
	Fuse12VPWM_SetTripTime(&WaterPump1, ms_200);
	Fuse12VPWM_SetCurrentLimit(&WaterPump1, A_11_5);
	Fuse12VPWM_SetInputFrequency(&WaterPump1, 500);
	Fuse12VPWM_SetInputDutyCycle(&WaterPump1, 0.5);
	Fuse12VPWM_StartPWM(&WaterPump1);

	Fuse12VPWM_SetTripTime(&WaterPump1, ms_200);
	Fuse12VPWM_SetCurrentLimit(&WaterPump1, A_11_5);
	Fuse12VPWM_SetInputFrequency(&WaterPump1, 500);
	Fuse12VPWM_SetInputDutyCycle(&WaterPump1, 0.5);
	Fuse12VPWM_StartPWM(&WaterPump1);
	HAL_Delay(150);

	/*Group 3*/
	HighPoweredFuse_SetEnable(&FuelPump, 1);
	HighPoweredFuse_SetFaultRST(&FuelPump, 0);
	HAL_Delay(150);

	/*Group 4*/
	HighPoweredFuse_SetEnable(&SpareHP, 1);
	HighPoweredFuse_SetFaultRST(&SpareHP, 0);
	HAL_Delay(150);

	/*Group 5*/
	RadioFuse_SetEnable(&Radio, 1);
	RadioFuse_SetCurrentLimit(&Radio, A_4);
	RadioFuse_SetTripTime(&Radio, ms_200);

	for(int i = 0; i < Number_12VFuses; i++)
	{
		Fuse12V_SetTripTime(Fuses12V[i], ms_200);
		Fuse12V_SetCurrentLimit(Fuses12V[i], A_2_5);
		Fuse12V_SetEnable(Fuses12V[i], 1);
	}

	for(int i = 0; i < Number_LPFuses; i++)
	{
		LowPowerFuse_SetEnable(LowPowerFuses[i], LowPowerFuse_ON);
	}

	osThreadSuspend(startupTaskHandle);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSamplingTask */
/**
* @brief Function implementing the samplingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSamplingTask */
void StartSamplingTask(void *argument)
{
  /* USER CODE BEGIN StartSamplingTask */
  /* Infinite loop */
  for(;;)
  {

  	float data;
  	for(int i = 0; i < Number_HPFuses; i++)
		{
  		data = HighPoweredFuse_GetSenseData(HighPoweredFuses[i]);
    	snprintf(TxData_Sense, sizeof(TxData_Sense), "%hhu %2.2f", HighPoweredFuses[i]->ID, data);
    	HAL_CAN_AddTxMessage(&hcan, &TxHeader_FuseCurrent, TxData_Sense, &canMailbox);
  		//TODO: Send data over CAN.
		}

  	osMutexAcquire(adc2MutexHandle, osWaitForever);
  	for(int i = 0; i < Number_12VPWMFuses; i++)
		{
  		data = Fuse12VPWM_GetCurrentSense(PWMedFuses[i]);
			snprintf(TxData_Sense, sizeof(TxData_Sense), "%hhu %2.2f", PWMedFuses[i]->ID, data);
			HAL_CAN_AddTxMessage(&hcan, &TxHeader_FuseCurrent, TxData_Sense, &canMailbox);
		}

  	for(int i = 0; i < Number_12VFuses; i++)
		{
  		data = Fuse12V_GetCurrentSense(Fuses12V[i]);
			snprintf(TxData_Sense, sizeof(TxData_Sense), "%hhu %2.2f", Fuses12V[i]->ID, data);
			HAL_CAN_AddTxMessage(&hcan, &TxHeader_FuseCurrent, TxData_Sense, &canMailbox);
		}
  	osMutexRelease(adc2MutexHandle);

  	data = VoltageSense_GetVoltage(&VSense12V);
		snprintf(TxData_Sense, sizeof(TxData_Sense), "%2.2f", data);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader_BatVoltage, TxData_Sense, &canMailbox);

  	//data = CurrentSense_GetCurrent(&CSense8V); //8V eFuse goes into fault with any kind fo significant load.
  	//TODO: Send data over CAN.

  	data = CurrentSense_GetCurrent(&CSense5V);
		snprintf(TxData_Sense, sizeof(TxData_Sense), "5 %2.2f", data);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader_RegCurrent, TxData_Sense, &canMailbox);

  	/*
  	snprintf(TxData, sizeof(TxData), "1 %2.2f", FanCurrent);
  	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &canMailbox);
  	osDelay(500);

  	snprintf(TxData, sizeof(TxData), "2 %2.2f", Regulator8VCurrent);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &canMailbox);
		osDelay(500);

		snprintf(TxData, sizeof(TxData), "3 %2.2f", SpareCurrent);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &canMailbox);
		osDelay(500);

		snprintf(TxData, sizeof(TxData), "4 %2.2f", BatteryVoltage);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &canMailbox);
		osDelay(500);

		snprintf(TxData, sizeof(TxData), "5 %2.2f", PumpCurrent);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &canMailbox);
		osDelay(500);

		snprintf(TxData, sizeof(TxData), "6 %2.2f", Regulator5VCurrent);
		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &canMailbox);
		osDelay(500);

  	char msg[1600] = {'\0'};
		for (int i = 0; i < 42; i++)
		{
			char buffer[24] = {'\0'};
			snprintf(buffer, sizeof(buffer), "Sample: %d : %2.3f\n\0", i, data[i]);
			strncat(msg, buffer, strlen(buffer));
		}
		printf(msg);
  	*/
  	osDelay(50);
  }
  /* USER CODE END StartSamplingTask */
}

/* USER CODE BEGIN Header_StartDiagnosticCheckTask */
/**
* @brief Function implementing the diagnosticCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDiagnosticCheckTask */
void StartDiagnosticCheckTask(void *argument)
{
  /* USER CODE BEGIN StartDiagnosticCheckTask */
  /* Infinite loop */
  for(;;)
  {
  	osMutexAcquire(adc2MutexHandle, osWaitForever);
  	for(int i = 0; i < Number_HPFuses; i++)
		{
  		if(HighPoweredFuse_IsEnabled(HighPoweredFuses[i]) && !PWMedFuses[i]->criticalFault)
			{
				if(HighPoweredFuse_IsFault(HighPoweredFuses[i]))
				{
					TxData_Faults[0] = HighPoweredFuses[i]->ID;
					HAL_CAN_AddTxMessage(&hcan, &TxHeader_Fault, TxData_Faults, &canMailbox);
					if(HighPoweredFuse_RetryProcedure(HighPoweredFuses[i]))
					{
						TxData_Faults[0] = HighPoweredFuses[i]->ID;
						HAL_CAN_AddTxMessage(&hcan, &TxHeader_CriticalFault, TxData_Faults, &canMailbox);
					}
				}
			}
		}

  	for(int i = 0; i < Number_12VPWMFuses; i++)
		{
  		if(Fuse12VPWM_IsEnabled(PWMedFuses[i]) && PWMedFuses[i]->dutyCycle > 0 && !PWMedFuses[i]->criticalFault)
			{
				if(Fuse12VPWM_IsFault(PWMedFuses[i]))
				{
					TxData_Faults[0] = PWMedFuses[i]->ID;
					HAL_CAN_AddTxMessage(&hcan, &TxHeader_Fault, TxData_Faults, &canMailbox);
					if(Fuse12VPWM_RetryProcedure(PWMedFuses[i]))
					{
						TxData_Faults[0] = PWMedFuses[i]->ID;
						HAL_CAN_AddTxMessage(&hcan, &TxHeader_CriticalFault, TxData_Faults, &canMailbox);
					}
				}
			}
		}

  	for(int i = 0; i < Number_12VFuses; i++)
		{
  		if(Fuse12V_IsEnabled(Fuses12V[i]) && !Fuses12V[i]->criticalFault)
			{
				if(Fuse12V_IsFault(Fuses12V[i]))
				{
					TxData_Faults[0] = Fuses12V[i]->ID;
					HAL_CAN_AddTxMessage(&hcan, &TxHeader_Fault, TxData_Faults, &canMailbox);
					if(Fuse12V_RetryProcedure(Fuses12V[i]))
					{
						TxData_Faults[0] = Fuses12V[i]->ID;
						HAL_CAN_AddTxMessage(&hcan, &TxHeader_CriticalFault, TxData_Faults, &canMailbox);
					}
				}
			}
		}

  	for(int i = 0; i < Number_LPFuses; i++)
		{
  		if(LowPowerFuse_IsEnabled(LowPowerFuses[i]) && !LowPowerFuses[i]->criticalFault)
			{
				if(LowPowerFuse_IsFault(LowPowerFuses[i]))
				{
					TxData_Faults[0] = LowPowerFuses[i]->ID;
					HAL_CAN_AddTxMessage(&hcan, &TxHeader_Fault, TxData_Faults, &canMailbox);
					if(LowPowerFuse_RetryProcedure(LowPowerFuses[i]))
					{
						TxData_Faults[0] = LowPowerFuses[i]->ID;
						HAL_CAN_AddTxMessage(&hcan, &TxHeader_CriticalFault, TxData_Faults, &canMailbox);
					}
				}
			}
		}

  	if(RadioFuse_IsEnabled(&Radio) && !Radio.criticalFault)
  	{
  		if(RadioFuse_IsFault(&Radio))
			{
				TxData_Faults[0] = LowPowerFuses[i]->ID;
				HAL_CAN_AddTxMessage(&hcan, &TxHeader_Fault, TxData_Faults, &canMailbox);
				if(RadioFuse_RetryProcedure(&Radio))
				{
					TxData_Faults[0] = LowPowerFuses[i]->ID;
					HAL_CAN_AddTxMessage(&hcan, &TxHeader_CriticalFault, TxData_Faults, &canMailbox);
				}
			}
  	}
		osMutexRelease(adc2MutexHandle);
    osDelay(20);
  }
  /* USER CODE END StartDiagnosticCheckTask */
}

/* USER CODE BEGIN Header_StartKickDogTask */
/**
* @brief Function implementing the kickDogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKickDogTask */
void StartKickDogTask(void *argument)
{
  /* USER CODE BEGIN StartKickDogTask */
  /* Infinite loop */
  for(;;)
  {
  	osDelay(240);
  	HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END StartKickDogTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
