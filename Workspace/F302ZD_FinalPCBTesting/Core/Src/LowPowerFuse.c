/*
 * LowPowerFuse.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
#include "LowPowerFuse.h"

void LowPowerFuse_SetEnable (LowPowerFuse *fuse, LowPowerFuse_EN state)
{
	HAL_GPIO_WritePin(fuse->port_input, fuse->pin_input, state);
	return;
}

GPIO_PinState LowPowerFuse_GetDiagnostic (LowPowerFuse *fuse)
{
	return HAL_GPIO_ReadPin(fuse->port_diagnostic, fuse->pin_diagnostic);
}

