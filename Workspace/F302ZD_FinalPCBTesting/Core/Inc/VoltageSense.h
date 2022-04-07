/*
 * VoltageSense.h
 *
 *  Created on: Apr 4, 2022
 *      Author: Guill
 */

#ifndef INC_VOLTAGESENSE_H_
#define INC_VOLTAGESENSE_H_

#include "main.h"

typedef struct
{
	ADC_HandleTypeDef *ADC_voltageSense;
	uint32_t ADC_channel;
	uint8_t mux_channel;

	float voltageDivision;
}VoltageSense;

float VoltageSense_GetVoltage(VoltageSense *signal);




#endif /* INC_VOLTAGESENSE_H_ */
