/*
 * Fuse12VPWM.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_FUSE12VPWM_H_
#define INC_FUSE12VPWM_H_
#include "main.h"
#include "Fuse12V.h"
#include "Fuse12VSettings.h"

typedef struct
{
	TIM_HandleTypeDef *TIM_input;
	uint32_t TIM_channel;
	float dutyCycle;
	uint16_t frequency;

	ADC_HandleTypeDef *ADC_diagnostic;
	uint32_t ADC_diagnosticChannel;
	uint8_t mux_diagnosticChannel;

	ADC_HandleTypeDef *ADC_currentSense;
	uint32_t ADC_currentSenseChannel;
	uint8_t mux_currentSenseChannel;

	uint16_t currentGain;
	uint16_t currentShunt;

	Fuse12VSettings settings;
}Fuse12V_PWM;

void Fuse12VPWM_SetTripTime(Fuse12V_PWM *fuse, Delay delay);
void Fuse12VPWM_SetCurrentLimit(Fuse12V_PWM *fuse, CurrentLimit limit);
void Fuse12VPWM_SetInputFrequency(Fuse12V_PWM *fuse, uint16_t frequency);
void Fuse12VPWM_SetInputDutyCycle(Fuse12V_PWM *fuse, float dutyCycle);
uint8_t Fuse12VPWM_IsFault(Fuse12V_PWM *fuse);

#endif /* INC_FUSE12VPWM_H_ */
