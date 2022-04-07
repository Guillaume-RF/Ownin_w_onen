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
#include "cmsis_os.h"

typedef struct
{
	uint8_t ID;

	GPIO_TypeDef *port_input;
	GPIO_TypeDef *port_diagnostic;

	uint16_t pin_input;
	uint16_t pin_diagnostic;

	uint32_t time_ms_lastRetryProcedure;
	uint8_t retries;
	uint8_t criticalFault;

	osTimerId_t retryTimer;

	Fuse12VSettings *settings;
}RadioFuse;

void RadioFuse_SetTripTime(RadioFuse *fuse, Delay delay);
void RadioFuse_SetCurrentLimit(RadioFuse *fuse, CurrentLimit limit);
void RadioFuse_SetEnable (RadioFuse *fuse, GPIO_PinState state);
GPIO_PinState RadioFuse_IsFault(RadioFuse *fuse);
uint8_t RadioFuse_IsEnabled(RadioFuse *fuse);
uint8_t RadioFuse_RetryProcedure(RadioFuse *fuse);

#endif /* INC_RADIOFUSE_H_ */
