/*
 * PWM.c
 *
 *  Created on: 02/12/2017
 *      Author: Roberto
 */
#include "PWM.h"
#include "FlexTimer.h"

const FTM_ConfigType FTM_Config = {
		FTM_0,
		WPDIS_DISABLE,
		FTM_DISABLE,
		CHANNEL_6,
		SYS_CLK,
		DIVIDER_128
};

void PWM_init(void){
	FlexTimer_Init(&FTM_Config);

}
