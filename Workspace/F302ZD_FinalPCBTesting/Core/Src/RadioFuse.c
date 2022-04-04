/*
 * RadioFuse.c
 *
 *  Created on: Apr 3, 2022
 *      Author: Guill
 */
#include "RadioFuse.h"

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

GPIO_PinState RadioFuse_GetDiagnostic(RadioFuse *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_diagnostic, fuse->pin_diagnostic);
}



