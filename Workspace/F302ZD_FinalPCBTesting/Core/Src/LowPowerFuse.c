/*
 * LowPowerFuse.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */
void LowPowerFuse_SetEnable (Fuse12V *fuse, LowPowerFuse_EN state)
{
	HAL_GPIO_WritePin(fuse->port_input, fuse->pin_input, state);
	return;
}

