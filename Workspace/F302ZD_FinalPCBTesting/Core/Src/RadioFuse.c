/*
 * RadioFuse.c
 *
 *  Created on: Apr 3, 2022
 *      Author: Guill
 */
#include "RadioFuse.h"
#include <stdio.h>

void RadioFuse_SetTripTime(RadioFuse *fuse, Delay delay)
{
	Fuse12VSettings_SetTripTime(fuse->settings, delay);
	return;
}

void RadioFuse_SetCurrentLimit(RadioFuse *fuse, CurrentLimit limit)
{
	Fuse12VSettings_SetCurrentLimit(fuse->settings, limit);
	return;
}

void RadioFuse_SetEnable(RadioFuse *fuse, GPIO_PinState state)
{
	HAL_GPIO_WritePin(fuse->port_input, fuse->pin_input, state);
	return;
}

GPIO_PinState RadioFuse_IsFault(RadioFuse *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_diagnostic, fuse->pin_diagnostic);
}

uint8_t RadioFuse_IsEnabled(RadioFuse *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_input, fuse->pin_input);
}

void RadioFuse_RetryCallback(void * argument)
{
	RadioFuse *fuse=(RadioFuse *)argument;
	RadioFuse_SetEnable(fuse, 1);
	osTimerDelete(fuse->retryTimer);
}

uint8_t RadioFuse_RetryProcedure(RadioFuse *fuse)
{
	RadioFuse_SetEnable(fuse, 0);
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
		fuse->retryTimer = osTimerNew(RadioFuse_RetryCallback, osTimerOnce, fuse, &attributes);
		osTimerStart(fuse->retryTimer, FUSE_RESTART_WAIT_PERIOD_MS);
		return 0;
	}
}



