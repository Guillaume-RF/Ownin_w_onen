/*
 * LowPowerFuse.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_LOWPOWERFUSE_H_
#define INC_LOWPOWERFUSE_H_

#include "main.h"
#include "cmsis_os.h"

typedef enum
{
	LowPowerFuse_ON = 0,
	LowPowerFuse_OFF = 1
}LowPowerFuse_EN;

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
}LowPowerFuse;

void LowPowerFuse_SetEnable (LowPowerFuse *fuse, LowPowerFuse_EN state);
GPIO_PinState LowPowerFuse_IsFault(LowPowerFuse *fuse);
uint8_t LowPowerFuse_RetryProcedure(LowPowerFuse *fuse);
uint8_t LowPowerFuse_IsEnabled(LowPowerFuse *fuse);

#endif /* INC_LOWPOWERFUSE_H_ */
