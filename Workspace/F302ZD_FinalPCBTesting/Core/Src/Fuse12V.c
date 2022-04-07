/*
 * Fuse12V.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "Fuse12V.h"
#include "PeripheralUtilities.h"
#include <stdio.h>
#include <string.h>

uint8_t Fuse12V_IsEnabled(Fuse12V *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_input, fuse->pin_input);
}

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
		return -1;
	}

	voltage = (float)raw / ADC_RES * ADC_VREF;
	current = voltage / fuse->currentGain / fuse->currentShunt;
	return current;
}

GPIO_PinState Fuse12V_IsFault(Fuse12V *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_diagnostic, fuse->pin_diagnostic);
}

void Fuse12V_RetryCallback(void * argument)
{
	Fuse12V *fuse=(Fuse12V *)argument;
	Fuse12V_SetEnable(fuse, 1);
	osTimerDelete(fuse->retryTimer);
}

//Begins retry procedure. Returns 1 if in critical fault, 0 otherwrise.
uint8_t Fuse12V_RetryProcedure(Fuse12V *fuse)
{
	Fuse12V_SetEnable(fuse, 0);
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
		fuse->retryTimer = osTimerNew(Fuse12V_RetryCallback, osTimerOnce, fuse, &attributes);
		osTimerStart(fuse->retryTimer, FUSE_RESTART_WAIT_PERIOD_MS);
		return 0;
	}
}


