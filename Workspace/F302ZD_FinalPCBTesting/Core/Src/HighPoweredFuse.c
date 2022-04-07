/*
 * HighPoweredFuse.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "HighPoweredFuse.h"
#include "PeripheralUtilities.h"
#include <stdio.h>

uint8_t HighPoweredFuse_IsEnabled(HighPoweredFuse *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_input, fuse->pin_input);
}

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

uint8_t HighPoweredFuse_IsFault(HighPoweredFuse *fuse)
{
	return HighPoweredFuse_GetSenseData(fuse) < 0;
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

void HighPoweredFuse_RetryCallback(void * argument)
{
	HighPoweredFuse *fuse=(HighPoweredFuse *)argument;
	HighPoweredFuse_SetEnable(fuse, 1);
	osTimerDelete(fuse->retryTimer);
}

//Begins retry procedure. Returns 1 if in critical fault, 0 otherwrise.
uint8_t HighPoweredFuse_RetryProcedure(HighPoweredFuse *fuse)
{
	HighPoweredFuse_SetEnable(fuse, 0);
	if ((HAL_GetTick() - fuse->time_ms_lastRetryProcedure) > FUSE_CRITICAL_FAULT_PERIOD_MS || fuse->time_ms_lastRetryProcedure == 0)
	{
		fuse->retries = 0;
		fuse->time_ms_lastRetryProcedure = HAL_GetTick();
	}
	fuse->retries++;
	if (fuse->retries > FUSE_RETRY_ATTEMPTS)
	{
		fuse->criticalFault = 1;
		return 1;
	}
	else
	{
		char name[10];
		sprintf(name, "%lu", fuse->time_ms_lastRetryProcedure);
		osTimerAttr_t attributes = {.name = name};
		fuse->retryTimer = osTimerNew(HighPoweredFuse_RetryCallback, osTimerOnce, fuse, &attributes);
		osTimerStart(fuse->retryTimer, FUSE_RESTART_WAIT_PERIOD_MS);
		return 0;
	}
}

