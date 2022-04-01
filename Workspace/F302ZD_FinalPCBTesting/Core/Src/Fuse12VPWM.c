/*
 * Fuse12VPWM.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "Fuse12VPWM.h"
#include "PeripheralUtilities.h"

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
	//TODO: May need a mutex around this method if duty cycle is changed through interrupt.
	TIM_SetPWMDutyCycle(fuse->TIM_input, fuse->TIM_channel, dutyCycle);
	fuse->dutyCycle = dutyCycle;
	return;
}

/*
 * Gets the current through the fuse. Since enable of this fuse can be PWM'ed, the current sense may also be
 * a PWM signal. Thus, we should sample enough within a certain interval to be sure to sample at least 1 'on'
 * period of this PWM current sense signal.
 */
float Fuse12VPWM_GetCurrentSense(Fuse12V_PWM *fuse)
{
	ADC_SetChannel(fuse->ADC_currentSense, fuse->ADC_currentSenseChannel);
	Mux_SetChannel(fuse->mux_currentSenseChannel);
	HAL_ADC_Start(fuse->ADC_currentSense);
	uint32_t raw = 0, maxRaw = 0, samples = 5;
	float voltage = 0, current = 0;

	if (fuse->dutyCycle == 100)	//TODO: May need a mutex if duty cycle is changed through interrupt.
	{
		samples = 1;
	}

	for (int i = 0; i < samples; i++)
	{
		if (HAL_ADC_PollForConversion(fuse->ADC_currentSense, 10) == HAL_OK)
		{
			raw = HAL_ADC_GetValue(fuse->ADC_currentSense);
			HAL_ADC_Stop(fuse->ADC_currentSense);
			if (raw > maxRaw)
			{
				maxRaw = raw;
			}
		}
		else
		{
			return 0;
		}
		Delay_us(200);
	}

	voltage = (float)maxRaw / ADC_RES * ADC_VREF;
	current = voltage / fuse->currentGain / fuse->currentShunt;
	return current;
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
