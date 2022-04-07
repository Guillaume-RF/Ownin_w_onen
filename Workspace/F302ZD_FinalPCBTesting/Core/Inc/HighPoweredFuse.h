/*
 * HighPoweredFuse.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_HIGHPOWEREDFUSE_H_
#define INC_HIGHPOWEREDFUSE_H_
#include "main.h"
#include "cmsis_os.h"

typedef enum
{
	Current = 0,
	ChipTemp = 2,
	VccSense = 3
}HighPoweredFuse_Sense;

typedef struct
{
	uint8_t ID;

	GPIO_TypeDef *port_input;
	GPIO_TypeDef *port_senseSelect0;
	GPIO_TypeDef *port_senseSelect1;
	GPIO_TypeDef *port_faultRST;

	uint16_t pin_input;
	uint16_t pin_senseSelect0;
	uint16_t pin_senseSelect1;
	uint16_t pin_faultRST;

	ADC_HandleTypeDef *ADC_multiSense;
	uint32_t ADC_channel;
	uint8_t mux_channel;

	HighPoweredFuse_Sense senseState;
	uint16_t currentGain;
	float currentShunt;

	uint32_t time_ms_lastRetryProcedure;
	uint8_t retries;
	uint8_t criticalFault;

	osTimerId_t retryTimer;
}HighPoweredFuse;

void HighPoweredFuse_SetEnable(HighPoweredFuse *fuse, GPIO_PinState state);
void HighPoweredFuse_SetFaultRST (HighPoweredFuse *fuse, GPIO_PinState state);
void HighPoweredFuse_SetSenseSelect (HighPoweredFuse *fuse, HighPoweredFuse_Sense state);
float HighPoweredFuse_GetSenseData (HighPoweredFuse *fuse);
uint8_t HighPoweredFuse_IsEnabled(HighPoweredFuse *fuse);
uint8_t HighPoweredFuse_IsFault(HighPoweredFuse *fuse);
uint8_t HighPoweredFuse_RetryProcedure(HighPoweredFuse *fuse);

#endif /* INC_HIGHPOWEREDFUSE_H_ */
