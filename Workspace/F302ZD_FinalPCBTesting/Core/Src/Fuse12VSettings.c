/*
 * Fuse12VSettings.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "Fuse12VSettings.h"

void Fuse12VSettings_SetTripTime(Fuse12VSettings *settings, Delay delay)
{
	uint8_t s0 = delay & 0x01;
	uint8_t s1 = delay & 0x02;

	HAL_GPIO_WritePin(settings->port_delayMux0, settings->pin_delayMux0, s0);
	HAL_GPIO_WritePin(settings->port_delayMux1, settings->pin_delayMux1, s1);
}

void Fuse12VSettings_SetCurrentLimit(Fuse12VSettings *settings, CurrentLimit limit)
{
	uint8_t s0 = limit & 0x01;
	uint8_t s1 = limit & 0x02;

	HAL_GPIO_WritePin(settings->port_currentMux0, settings->pin_currentMux0, s0);
	HAL_GPIO_WritePin(settings->port_currentMux1, settings->pin_currentMux1, s1);
}

