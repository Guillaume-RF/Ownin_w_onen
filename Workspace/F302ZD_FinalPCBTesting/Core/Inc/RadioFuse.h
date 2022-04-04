/*
 * RadioFuse.h
 *
 *  Created on: Apr 3, 2022
 *      Author: Guill
 */

#ifndef INC_RADIOFUSE_H_
#define INC_RADIOFUSE_H_

#include "main.h"
#include "Fuse12VSettings.h"

typedef struct
{
	GPIO_TypeDef *port_input;
	GPIO_TypeDef *port_diagnostic;

	uint16_t pin_input;
	uint16_t pin_diagnostic;

	Fuse12VSettings *settings;
}RadioFuse;

void RadioFuse_SetTripTime(RadioFuse *fuse, Delay delay);
void RadioFuse_SetCurrentLimit(RadioFuse *fuse, CurrentLimit limit);
void RadioFuse_SetEnable (RadioFuse *fuse, GPIO_PinState state);
GPIO_PinState RadioFuse_GetDiagnostic(RadioFuse *fuse);

#endif /* INC_RADIOFUSE_H_ */
