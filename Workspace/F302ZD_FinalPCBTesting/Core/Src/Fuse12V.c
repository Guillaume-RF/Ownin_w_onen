/*
 * Fuse12V.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "Fuse12V.h"
#include "PeripheralUtilities.h"

void Fuse12V_SetTripTime(Fuse12V *fuse, Delay delay)
{
	Fuse12VSettings_SetTripTime(fuse->settings, delay);
	return;
}

void Fuse12V_SetCurrentLimit(Fuse12V *fuse, CurrentLimit limit)
{
	Fuse12VSettings_SetCurrentLimit(fuse->settings, limit);
	return;
}

void Fuse12V_SetEnable(Fuse12V *fuse, GPIO_PinState state)
{
	HAL_GPIO_WritePin(fuse->port_input, fuse->pin_input, state);
	return;
}

float Fuse12V_GetCurrentSense(Fuse12V *fuse)
{
	ADC_SetChannel(fuse->ADC_currentSense, fuse->ADC_channel);
	Mux_SetChannel(fuse->mux_channel);
	HAL_ADC_Start(fuse->ADC_currentSense);
	uint32_t raw;
	float voltage = 0, current = 0;

	if (HAL_ADC_PollForConversion(fuse->ADC_currentSense, 10) == HAL_OK)
	{
		raw = HAL_ADC_GetValue(fuse->ADC_currentSense);
		HAL_ADC_Stop(fuse->ADC_currentSense);
	}
	else
	{
		return 0;
	}

	voltage = (float)raw / ADC_RES;
	current = voltage / fuse->currentGain / fuse->currentShunt;
	return current;
}


