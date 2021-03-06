/*
 * PeripheralUtilities.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_PERIPHERALUTILITIES_H_
#define INC_PERIPHERALUTILITIES_H_
#include "main.h"

TIM_HandleTypeDef TIM4_delay; //Timer 4 being used in delay method.

void Mux_SetChannel(uint8_t channel);
void ADC_SetChannel (ADC_HandleTypeDef *adc, uint32_t channel);
void TIM_SetPWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t frequency, float currentDutyCycle);
void TIM_SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float dutyCycle);
void TIM4_Init(void);
void Delay_us(uint16_t us);
#endif /* INC_PERIPHERALUTILITIES_H_ */
