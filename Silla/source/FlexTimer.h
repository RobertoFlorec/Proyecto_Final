/**
	\file
	\brief
		This is the header file for the FlexTimer divice driver.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Add configuration structures.
 */

#ifndef FLEXTIMER_H_
#define FLEXTIMER_H_


#include "MK64F12.h"
#include "DataTypeDefinitions.h"


#define FLEX_TIMER_0_CLOCK_GATING 0x01000000
#define FLEX_TIMER_1_CLOCK_GATING 0x02000000
#define FLEX_TIMER_2_CLOCK_GATING 0x04000000
#define FLEX_TIMER_3_CLOCK_GATING 0x02000000

#define FLEX_TIMER_FAULTIE  0x80
#define FLEX_TIMER_FAULTM_0   0x00
#define FLEX_TIMER_FAULTM_1   0x20
#define FLEX_TIMER_FAULTM_2   0x40
#define FLEX_TIMER_FAULTM_3   0x60
#define FLEX_TIMER_CAPTEST  0x10
#define FLEX_TIMER_PWMSYNC  0x08
#define FLEX_TIMER_WPDIS    0x04
#define FLEX_TIMER_INIT     0x02
#define FLEX_TIMER_FTMEN    0x01

#define FLEX_TIMER_TOF     0x80
#define FLEX_TIMER_TOIE    0x40
#define FLEX_TIMER_CPWMS   0x20



#define FLEX_TIMER_PWMLOAD_CH0 0x01
#define FLEX_TIMER_PWMLOAD_CH1 0x02
#define FLEX_TIMER_PWMLOAD_CH2 0x04
#define FLEX_TIMER_PWMLOAD_CH3 0x08
#define FLEX_TIMER_PWMLOAD_CH4 0x10
#define FLEX_TIMER_PWMLOAD_CH5 0x20
#define FLEX_TIMER_PWMLOAD_CH6 0x40
#define FLEX_TIMER_PWMLOAD_CH7 0x80
#define FLEX_TIMER_LDOK        0x200


#define  FLEX_TIMER_DMA   0x01
#define  FLEX_TIMER_ELSA  0x04
#define  FLEX_TIMER_ELSB  0x08
#define  FLEX_TIMER_MSA   0x10
#define  FLEX_TIMER_MSB   0x20
#define  FLEX_TIMER_CHIE  0x40
#define  FLEX_TIMER_CHF   0x80

typedef enum{
	FTM_0,
	FTM_1,
	FTM_2,
	FTM_3
}FTM_ChannelType;

typedef enum{
	WPDIS_ENABLE,
	WPDIS_DISABLE
}WPDIS_selectStateType;

typedef enum{
	FTM_ENABLE,
	FTM_DISABLE
}FTM_selectStateType;

typedef enum{
	CHANNEL_0,
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	CHANNEL_5,
	CHANNEL_6,
	CHANNEL_7
}channelType_FTM;

typedef enum{
	NO_CLK = 0x00,
	SYS_CLK = 0x08,
	FXD_CLK = 0x10,
	EXT_CLK = 0x18
}clk_selectType;


typedef enum{
	DIVIDER_1 = 0x00,
	DIVIDER_2,
	DIVIDER_4,
	DIVIDER_8,
	DIVIDER_16,
	DIVIDER_32,
	DIVIDER_64,
	DIVIDER_128
}DividerType;

typedef struct{
	FTM_ChannelType FTM_Channel;
	WPDIS_selectStateType FTM_WPDIS;
	FTM_selectStateType FTM_Status;
	channelType_FTM channel;
	clk_selectType FTM_CLK;
	DividerType FTM_DIVIDER;
}FTM_ConfigType;

void FlexTimer_clockGating(FTM_ChannelType Channel);
void FlexTimerWPDIS(WPDIS_selectStateType select);
void FlexTimer_MODSelect(FTM_ChannelType Channel);
void FTM_enable(FTM_selectStateType select);
void FlexTimer_setChannel(FTM_ChannelType Channel, channelType_FTM chan);
void FlexTimer_CnVset(FTM_ChannelType Channel, channelType_FTM chan);
void FlexTimer_setClk(FTM_ChannelType Channel, clk_selectType FLEX_TIMER_CLKS, DividerType FLEX_TIMER_PS);
void FlexTimer_updateCHValue(sint16 channelValue, FTM_ChannelType Channel, uint8 chan);

void FlexTimer_Init(const FTM_ConfigType* FTMconfig);


#endif /* FLEXTIMER_H_ */
