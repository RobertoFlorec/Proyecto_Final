/*
 * ACELEROMETRO.c
 *
 *  Created on: 27/11/2017
 *      Author: Roberto
 */
#include "ADC.h"
#include "ACELEROMETRO.h"
#include "PID.h"

const ADC_ConfigType ADC_Config = {
		ADC_0,
		BITMODE_8_9,
		DADP1,
		SAMPLES_4,
		AVERAGE_ENABLE,
		INTERRUPT_ENABLE
};

void acelerometroInit(void){
ADC_init(&ADC_Config);
}

uint8 getValue_Y(void){
	uint8 getValue_Y;
	getValue_Y = ADC_read(ADC_Config.ADC_Channel);
	return getValue_Y;
}

uint8 getState(void){
	uint32 PID_output;
	PID_output = PID();
	if ((PID_output > 0) && (PID_output <= 26)){
		return 1;
	}
	else if ((PID_output >= 232) && (PID_output <= 255)){
		return 2;
	}
	else return 0;
}
