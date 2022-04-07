/*
 * LowPowerFuse.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "LowPowerFuse.h"
#include <stdio.h>

uint8_t LowPowerFuse_IsEnabled(LowPowerFuse *fuse)
{
	return !HAL_GPIO_ReadPin(fuse->port_input, fuse->pin_input);
}

void LowPowerFuse_SetEnable (LowPowerFuse *fuse, LowPowerFuse_EN state)
{
	HAL_GPIO_WritePin(fuse->port_input, fuse->pin_input, state);
	return;
}

GPIO_PinState LowPowerFuse_IsFault (LowPowerFuse *fuse)
{
	return !HAL_GPIO_ReadPin(fuse->port_diagnostic, fuse->pin_diagnostic);
}

void LowPowerFuse_RetryCallback(void * argument)
{
	LowPowerFuse *fuse=(LowPowerFuse *)argument;
	LowPowerFuse_SetEnable(fuse, LowPowerFuse_ON);
	osTimerDelete(fuse->retryTimer);
}

//Begins retry procedure. Returns 1 if in critical fault, 0 otherwrise.
uint8_t LowPowerFuse_RetryProcedure(LowPowerFuse *fuse)
{
	LowPowerFuse_SetEnable(fuse, LowPowerFuse_OFF);
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
		fuse->retryTimer = osTimerNew(LowPowerFuse_RetryCallback, osTimerOnce, fuse, &attributes);
		osTimerStart(fuse->retryTimer, FUSE_RESTART_WAIT_PERIOD_MS);
		return 0;
	}
}


