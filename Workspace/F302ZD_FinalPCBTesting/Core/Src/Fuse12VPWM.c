/*
 * Fuse12VPWM.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "Fuse12VPWM.h"

void Fuse12VPWM_SetTripTime(Fuse12V_PWM *fuse, Delay delay)
{
	Fuse12VSettings_SetTripTime(fuse->settings, delay);
	return;
}

void Fuse12VPWM_SetCurrentLimit(Fuse12V_PWM *fuse, CurrentLimit limit)
{
	Fuse12VSettings_SetCurrentLimit(fuse->settings, limit);
	return;
}

void Fuse12VPWM_SetInputFrequency(Fuse12V_PWM *fuse, uint16_t frequency)
{
	TIM_SetPWMFrequency(fuse->TIM_input, fuse->TIM_channel, frequency, fuse->dutyCycle);
	return;
}

void Fuse12VPWM_SetInputDutyCycle(Fuse12V_PWM *fuse, float dutyCycle)
{
	TIM_SetPWMDutyCycle(fuse->TIM_input, fuse->TIM_channel, dutyCycle);
	fuse->dutyCycle = dutyCycle;
	return;
}

/*
 * Returns 1 if the fuse is showing a fault.
 */
uint8_t Fuse12VPWM_IsFault(Fuse12V_PWM *fuse)
{
	ADC_SetChannel(fuse->ADC_diagnostic, fuse->ADC_diagnosticChannel);
	Mux_SetChannel(fuse->mux_diagnosticChannel);
	HAL_ADC_Start(fuse->ADC_diagnostic);
	uint32_t raw;

	if (HAL_ADC_PollForConversion(fuse->ADC_diagnostic, 10) == HAL_OK)
	{
		raw = HAL_ADC_GetValue(fuse->ADC_diagnostic);
		HAL_ADC_Stop(fuse->ADC_diagnostic);
	}
	else
	{
		return 0;
	}

	if (raw > (ADC_RES * 0.7) )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
