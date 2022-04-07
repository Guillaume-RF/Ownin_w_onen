/*
 * CurrentSense.c
 *
 *  Created on: Apr 4, 2022
 *      Author: Guill
 */
#include "CurrentSense.h"
#include "PeripheralUtilities.h"

float CurrentSense_GetCurrent(CurrentSense *currentSignal)
{
	ADC_SetChannel(currentSignal->ADC_currentSense, currentSignal->ADC_channel);
	Mux_SetChannel(currentSignal->mux_channel);
	HAL_ADC_Start(currentSignal->ADC_currentSense);
	uint32_t raw;
	float voltage = 0, current = 0;

	if (HAL_ADC_PollForConversion(currentSignal->ADC_currentSense, 10) == HAL_OK)
	{
		raw = HAL_ADC_GetValue(currentSignal->ADC_currentSense);
		HAL_ADC_Stop(currentSignal->ADC_currentSense);
	}
	else
	{
		return -1.0;
	}

	voltage = (float)raw / ADC_RES * ADC_VREF;
	current = voltage / currentSignal->currentGain / currentSignal->currentShunt;
	return current;
}



