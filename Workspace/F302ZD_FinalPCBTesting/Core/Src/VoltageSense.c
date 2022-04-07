/*
 * VoltageSense.c
 *
 *  Created on: Apr 4, 2022
 *      Author: Guill
 */
#include "VoltageSense.h"
#include "PeripheralUtilities.h"

float VoltageSense_GetVoltage(VoltageSense *signal)
{
	ADC_SetChannel(signal->ADC_voltageSense, signal->ADC_channel);
	Mux_SetChannel(signal->mux_channel);
	HAL_ADC_Start(signal->ADC_voltageSense);
	uint32_t raw;
	float voltage = 0;

	if (HAL_ADC_PollForConversion(signal->ADC_voltageSense, 10) == HAL_OK)
	{
		raw = HAL_ADC_GetValue(signal->ADC_voltageSense);
		HAL_ADC_Stop(signal->ADC_voltageSense);
	}
	else
	{
		return -1;
	}

	voltage = (float)raw / ADC_RES * ADC_VREF * signal->voltageDivision;
	return voltage;
}




