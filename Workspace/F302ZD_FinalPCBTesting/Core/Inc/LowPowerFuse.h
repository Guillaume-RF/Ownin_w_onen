/*
 * LowPowerFuse.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Guill
 */

#ifndef INC_LOWPOWERFUSE_H_
#define INC_LOWPOWERFUSE_H_

typedef enum
{
	LowPowerFuse_ON = 0,
	LowPowerFuse_OFF = 1
}LowPowerFuse_EN;

typedef struct
{
	GPIO_TypeDef *port_input;
	GPIO_TypeDef *port_diagnostic;

	uint16_t pin_input;
	uint16_t pin_diagnostic;
}LowPowerFuse;

void LowPowerFuse_SetEnable (Fuse12V *fuse, LowPowerFuse_EN state);

#endif /* INC_LOWPOWERFUSE_H_ */
