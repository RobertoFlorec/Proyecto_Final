/*
 * PWM.c
 *
 *  Created on: 02/12/2017
 *      Author: Roberto
 */
#include "PWM.h"
#include "FlexTimer.h"
#include "GPIO.h"

const FTM_ConfigType FTM_Config = {
		FTM_0,
		WPDIS_DISABLE,
		FTM_DISABLE,
		CHANNEL_6,
		SYS_CLK,
		DIVIDER_128
};



void PWM_init(void){

	GPIO_pinControlRegisterType	pinControlRegisterPORTA = GPIO_MUX3;
	/**Clock gating for port A and C*/
	GPIO_clockGating(GPIO_A);
	GPIO_pinControlRegister(GPIO_A, BIT1, &pinControlRegisterPORTA);


	/*******************************************************************/
	FlexTimer_Init(&FTM_Config);

}
