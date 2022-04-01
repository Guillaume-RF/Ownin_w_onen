/*
 * Fuse12VSettings.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_FUSE12VSETTINGS_H_
#define INC_FUSE12VSETTINGS_H_
#include "main.h"

typedef enum
{
	ms_200 = 0,	//200ms
	ms_50 = 1,	//50ms
	us_20 = 2,	//20us
	disable = 3	//Disable delay and current limit.
}Delay;

typedef enum
{
	A_11_5 = 0, //11.5A
	A_7 = 1,	//7A
	A_4 = 2,	//4A
	A_2_5 = 3	//2.5A
}CurrentLimit;

typedef struct
{
	GPIO_TypeDef *port_currentMux0;
	GPIO_TypeDef *port_currentMux1;
	GPIO_TypeDef *port_delayMux0;
	GPIO_TypeDef *port_delayMux1;

	uint16_t pin_currentMux0;
	uint16_t pin_currentMux1;
	uint16_t pin_delayMux0;
	uint16_t pin_delayMux1;

}Fuse12VSettings;

void Fuse12VSettings_SetCurrentLimit(Fuse12VSettings settings, CurrentLimit limit);
void Fuse12VSettings_SetTripTime(Fuse12VSettings settings, Delay delay);
#endif /* INC_FUSE12VSETTINGS_H_ */
