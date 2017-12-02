/**
	\file
	\brief
		This is the starter file of FlexTimer.
		In this file the FlexTimer is configured in overflow mode.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Add configuration structures.
 */

#include "FlexTimer.h"
#include "MK64F12.h"
#include <stdio.h>


void FTM0_ISR()
{
	printf("\nFTM0_ISR");
	FTM0->SC &= ~FLEX_TIMER_TOF;
	GPIOD->PDOR ^= 0xFF;
}

void FlexTimer_updateCHValue(sint16 channelValue,FTM_ChannelType Channel, uint8 chan){
	/**Assigns a new value for the duty cycle*/
	switch(Channel){
	case FTM_0:
		FTM0->CONTROLS[chan].CnV = channelValue;
		break;
	case FTM_1:
		FTM1->CONTROLS[chan].CnV = channelValue;
		break;
	case FTM_2:
		FTM2->CONTROLS[chan].CnV = channelValue;
		break;
	case FTM_3:
		FTM3->CONTROLS[chan].CnV = channelValue;
		break;
	default:
		break;
	}
}


void FlexTimer_Init(const FTM_ConfigType* FTMconfig){

		FlexTimer_clockGating(FTMconfig->FTM_Channel);/*! Clock gating for the FlexTimer 0*/
		FlexTimerWPDIS(FTMconfig->FTM_WPDIS);/*! When write protection is enabled (WPDIS = 0), write protected bits cannot be written. When write protection is disabled (WPDIS = 1), write protected bits can be written.*/
		FTM_enable(FTMconfig->FTM_Status);/*! Enables the writing over all registers*/
		FlexTimer_MODSelect(FTMconfig->FTM_Channel);/*! Assigning a default value for modulo register*/
		FlexTimer_setChannel(FTMconfig->FTM_Channel, FTMconfig->channel);/*! Selects the Edge-Aligned PWM mode mode*/
		FlexTimer_CnVset(FTMconfig->FTM_Channel, FTMconfig->channel);/*! Assign a duty cycle of 40%*/
		FlexTimer_setClk(FTMconfig->FTM_Channel, FTMconfig->FTM_CLK, FTMconfig->FTM_DIVIDER);/*! Configure the clock*/
}

void FlexTimer_clockGating(FTM_ChannelType Channel){
	switch(Channel){
	case FTM_0:
		SIM->SCGC6 |= FLEX_TIMER_0_CLOCK_GATING;
		break;
	case FTM_1:
		SIM->SCGC6 |= FLEX_TIMER_1_CLOCK_GATING;
		break;
	case FTM_2:
		SIM->SCGC6 |= FLEX_TIMER_2_CLOCK_GATING;
		break;
	case FTM_3:
		SIM->SCGC3 |= FLEX_TIMER_3_CLOCK_GATING;
		break;
	default:
		break;
	}
}

void FlexTimerWPDIS(WPDIS_selectStateType select){
	switch(select){
	case WPDIS_ENABLE:
		FTM0->MODE &= FLEX_TIMER_WPDIS;
		break;
	case WPDIS_DISABLE:
		FTM0->MODE |= FLEX_TIMER_WPDIS;
		break;
	default:
		break;
	}
}

void FTM_enable(FTM_selectStateType select){
	switch(select){
	case FTM_ENABLE:
		FTM0->MODE |= FLEX_TIMER_FTMEN;
		break;
	case FTM_DISABLE:
		FTM0->MODE &= ~FLEX_TIMER_FTMEN;
		break;
	default:
		break;
	}

}

void FlexTimer_MODSelect(FTM_ChannelType Channel){
	switch(Channel){
	case FTM_0:
		FTM0->MOD = 0x00FF;
		break;
	case FTM_1:
		FTM1->MOD = 0x00FF;
		break;
	case FTM_2:
		FTM2->MOD = 0x00FF;
		break;
	case FTM_3:
		FTM3->MOD = 0x00FF;
		break;
	default:
		break;
	}

}

void FlexTimer_setChannel(FTM_ChannelType Channel, channelType_FTM chan){
	switch(Channel){
	case FTM_0:
		FTM0->CONTROLS[chan].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case FTM_1:
		FTM1->CONTROLS[chan].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case FTM_2:
		FTM2->CONTROLS[chan].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case FTM_3:
		FTM3->CONTROLS[chan].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	default:
		break;
	}

}

void FlexTimer_CnVset(FTM_ChannelType Channel, channelType_FTM chan){
	switch(Channel){
		case FTM_0:
			FTM0->CONTROLS[chan].CnV = FTM0->MOD * 0.53;
			break;
		case FTM_1:
			FTM1->CONTROLS[chan].CnV = FTM1->MOD * 0.53;
			break;
		case FTM_2:
			FTM1->CONTROLS[chan].CnV = FTM2->MOD * 0.53;
			break;
		case FTM_3:
			FTM1->CONTROLS[chan].CnV = FTM3->MOD * 0.53;
			break;
		default:
			break;
		}
}

void FlexTimer_setClk(FTM_ChannelType Channel, clk_selectType FLEX_TIMER_CLKS, DividerType FLEX_TIMER_PS){
	switch(Channel){
	case FTM_0:
		FTM0->SC = FLEX_TIMER_CLKS|FLEX_TIMER_PS;
		break;
	case FTM_1:
		FTM1->SC = FLEX_TIMER_CLKS|FLEX_TIMER_PS;
		break;
	case FTM_2:
		FTM1->SC = FLEX_TIMER_CLKS|FLEX_TIMER_PS;
		break;
	case FTM_3:
		FTM1->SC = FLEX_TIMER_CLKS|FLEX_TIMER_PS;
		break;
	default:
		break;
	}

}
