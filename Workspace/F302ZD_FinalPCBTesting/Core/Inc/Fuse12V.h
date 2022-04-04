/*
 * Fuse12V.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_FUSE12V_H_
#define INC_FUSE12V_H_
#include "main.h"
#include "Fuse12VSettings.h"

typedef struct
{
	GPIO_TypeDef *port_input;
	GPIO_TypeDef *port_diagnostic;

	uint16_t pin_input;
	uint16_t pin_diagnostic;

	ADC_HandleTypeDef *ADC_currentSense;
	uint32_t ADC_channel;
	uint8_t mux_channel;

	uint16_t currentGain;
	uint16_t currentShunt;

	Fuse12VSettings *settings;
}Fuse12V;

void Fuse12V_SetTripTime(Fuse12V *fuse, Delay delay);
void Fuse12V_SetCurrentLimit(Fuse12V *fuse, CurrentLimit limit);
void Fuse12V_SetEnable (Fuse12V *fuse, GPIO_PinState state);
float Fuse12V_GetCurrentSense(Fuse12V *fuse);
GPIO_PinState Fuse12V_GetDiagnostic(Fuse12V *fuse);


#endif /* INC_FUSE12V_H_ */
