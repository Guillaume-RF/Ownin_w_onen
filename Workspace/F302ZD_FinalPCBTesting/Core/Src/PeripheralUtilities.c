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

