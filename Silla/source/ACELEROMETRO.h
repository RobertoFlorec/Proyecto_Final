/*
 * ACELEROMETRO.h
 *
 *  Created on: 27/11/2017
 *      Author: Roberto
 */

#ifndef ACELEROMETRO_H_
#define ACELEROMETRO_H_
#include "ADC.h"

typedef enum{
	STRAIGHT,
	UP,
	DOWN
}AxlDirection;

void acelerometroInit(void);
uint8 getValue_Y(void);
uint8 getState(void);

#endif /* ACELEROMETRO_H_ */
