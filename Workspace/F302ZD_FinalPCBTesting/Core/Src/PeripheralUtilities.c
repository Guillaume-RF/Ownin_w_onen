/*
 * PeripheralUtilities.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "PeripheralUtilities.h"

void ADC_SetChannel (ADC_HandleTypeDef *adc, uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(adc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void Mux_SetChannel(uint8_t channel)
{
	uint8_t s0 = channel & 0x01;
	uint8_t s1 = channel & 0x02;
	uint8_t s2 = channel & 0x04;

	HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, s0);
	HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, s1);
	HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, s2);
}

/*
 * Sets the frequency of PWM signal on timer without affecting the already set duty cycle.
 * This assumes timer in argument is connected to PCLK1.
 */
void TIM_SetPWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t frequency, float currentDutyCycle)
{
	uint32_t clk = HAL_RCC_GetPCLK1Freq();
	uint16_t psc = __HAL_TIM_GET_ICPRESCALER(htim, channel) - 1;
	uint16_t new_arr = (clk / ((psc + 1) * frequency)) - 1;
	uint16_t new_compare = (uint16_t)(currentDutyCycle * new_arr);

	__HAL_TIM_SET_AUTORELOAD(htim, new_arr);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, new_compare);
}

void TIM_SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float dutyCycle)
{
	uint16_t new_compare = __HAL_TIM_GET_AUTORELOAD(htim) * dutyCycle;
	__HAL_TIM_SET_COMPARE(htim, channel, new_compare);
}

void Delay_us(uint16_t us)
{
	/*
	TIM_HandleTypeDef test = TIM4_delay;
	__HAL_TIM_SET_COUNTER(&TIM4_delay, 0);
	HAL_TIM_Base_Start(&TIM4_delay);
	while (__HAL_TIM_GET_COUNTER(&TIM4_delay) < us)
	{
		int test2 = 0;
	}
	HAL_TIM_Base_Stop(&TIM4_delay);
	return;
	*/
	TIM_HandleTypeDef test = htim4;
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Base_Start(&htim4);
	while (__HAL_TIM_GET_COUNTER(&htim4) < us)
	{
		int test2 = 0;
	}
	HAL_TIM_Base_Stop(&htim4);
	return;
}

void TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  TIM4_delay.Instance = TIM4;
  TIM4_delay.Init.Prescaler = 8-1;
  TIM4_delay.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM4_delay.Init.Period = 0xffff-1;
  TIM4_delay.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM4_delay.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TIM4_delay) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM4_delay, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM4_delay, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
