/*
 * CurrentSense.h
 *
 *  Created on: Apr 4, 2022
 *      Author: Guill
 */

#ifndef INC_CURRENTSENSE_H_
#define INC_CURRENTSENSE_H_

#include "main.h"

typedef struct
{
	ADC_HandleTypeDef *ADC_currentSense;
	uint32_t ADC_channel;
	uint8_t mux_channel;

	uint16_t currentGain;
	float currentShunt;
}CurrentSense;

float CurrentSense_GetCurrent(CurrentSense *currentSignal);



#endif /* INC_CURRENTSENSE_H_ */
