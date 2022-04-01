/*
 * HighPoweredFuse.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "HighPoweredFuse.h"
#include "PeripheralUtilities.h"

void HighPoweredFuse_SetEnable(HighPoweredFuse *fuse, GPIO_PinState state)
{
	HAL_GPIO_WritePin(fuse->port_input, fuse->pin_input, state);
	return;
}

void HighPoweredFuse_SetFaultRST (HighPoweredFuse *fuse, GPIO_PinState state)
{
	HAL_GPIO_WritePin(fuse->port_faultRST, fuse->pin_faultRST, state);
	return;
}

/*
 * Selects the desired diagnostic output signal for the selected fuse.
 */
void HighPoweredFuse_SetSenseSelect(HighPoweredFuse *fuse, HighPoweredFuse_Sense state)
{
	switch (state)
	{
		case Current:
			HAL_GPIO_WritePin(fuse->port_senseSelect1, fuse->pin_senseSelect1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(fuse->port_senseSelect0, fuse->pin_senseSelect0, GPIO_PIN_RESET);
			fuse->senseState = Current;
			break;
		case ChipTemp:
			HAL_GPIO_WritePin(fuse->port_senseSelect1, fuse->pin_senseSelect1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(fuse->port_senseSelect0, fuse->pin_senseSelect0, GPIO_PIN_RESET);
			fuse->senseState = ChipTemp;
			break;
		case VccSense:
			HAL_GPIO_WritePin(fuse->port_senseSelect1, fuse->pin_senseSelect1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(fuse->port_senseSelect0, fuse->pin_senseSelect0, GPIO_PIN_SET);
			fuse->senseState = VccSense;
			break;
		default:
			HAL_GPIO_WritePin(fuse->port_senseSelect1, fuse->pin_senseSelect1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(fuse->port_senseSelect0, fuse->pin_senseSelect0, GPIO_PIN_RESET);
			fuse->senseState = Current;
			break;
	}
}

/*
 * Returns interpreted data from multi sense output. Returns -1.0 if in fault state.
 */
float HighPoweredFuse_GetSenseData(HighPoweredFuse *fuse)
{
	ADC_SetChannel(fuse->ADC_multiSense, fuse->ADC_channel);
	Mux_SetChannel(fuse->mux_channel);
	HAL_ADC_Start(fuse->ADC_multiSense);
	uint32_t raw;
	float voltage = 0, value = 0;

	if (HAL_ADC_PollForConversion(fuse->ADC_multiSense, 10) == HAL_OK)
	{
		raw = HAL_ADC_GetValue(fuse->ADC_multiSense);
		HAL_ADC_Stop(fuse->ADC_multiSense);
	}

	//ADC saturation is indicative of a fault.
	if (raw > (ADC_RES * 0.95))
	{
		return -1;
	}

	switch(fuse->senseState)
	{
		case Current:
			voltage = (float)raw / ADC_RES * ADC_VREF;
			value = voltage * fuse->currentGain / fuse->currentShunt;
			break;
		default:
			break;
		//TODO: Interpret other types of raw readings.
	}
	return value;
}

