/*
 * NextCentury Firmware
 * Copyright (C) 2014 - 2017. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted.
 *
 * Created: 4/11/2017
 *  Author: A. Paul & K. Paul
 */


#ifndef TDC_H_
#define TDC_H_

//=========================================================================
// Notes
//=========================================================================


//=========================================================================
// Definitions
//=========================================================================


//=========================================================================
// Includes
//=========================================================================
#include "Spi.h"

//=========================================================================
// Types
//=========================================================================
typedef enum TdcStatus_t
{
	TDC_UNINITILIZED,
	TDC_NORMAL,
	TDC_ERROR,
	TDC_SUCCESS,
}TdcStatus_t;

typedef struct __attribute__((packed)) tdcTimerRegs_t
{
	uint32_t				Timer1;
	uint32_t				Clock1;
	uint32_t				Timer2;
	uint32_t				Clock2;
	uint32_t				Timer3;
	uint32_t				Clock3;
	uint32_t				Timer4;
	uint32_t				Clock4;
	uint32_t				Timer5;
	uint32_t				Clock5;
	uint32_t				Timer6;
	uint32_t				Calb1;
	uint32_t				Calb2;
} tdcTimerRegs_t;

/*CONFIG1 REG*/
#define TDC_REG_CONFIG1							0x00
#define TDC_CONFIG1_FORCE_CAL_SHIFT				7
#define TDC_CONFIG1_FORCE_CAL_DISABLE			(0 << TDC_CONFIG1_FORCE_CAL_SHIFT)
#define TDC_CONFIG1_FORCE_CAL_ENABLE			(1 << TDC_CONFIG1_FORCE_CAL_SHIFT)
#define TDC_CONFIG1_PARITY_EN_SHIFT				6
#define DC_CONFIG1_PARITY_EN_DISABLED			(0 << TDC_CONFIG1_PARITY_EN_SHIFT)
#define DC_CONFIG1_PARITY_EN_ENABLED			(1 << TDC_CONFIG1_PARITY_EN_SHIFT)
#define TDC_CONFIG1_TRIGG_EDGE_SHIFT			5
#define DC_CONFIG1_TRIGG_EDGE_RISING			(0 << TDC_CONFIG1_TRIGG_EDGE_SHIFT)
#define DC_CONFIG1_TRIGG_EDGE_FALLING			(1 << TDC_CONFIG1_TRIGG_EDGE_SHIFT)
#define TDC_CONFIG1_STOP_EDGE_SHIFT				4
#define TDC_CONFIG1_STOP_EDGE_RISING			(0 << TDC_CONFIG1_STOP_EDGE_SHIFT)
#define TDC_CONFIG1_STOP_EDGE_FALLING			(1 << TDC_CONFIG1_STOP_EDGE_SHIFT)
#define TDC_CONFIG1_START_EDGE_SHIFT			3
#define TDC_CONFIG1_START_EDGE_RISING			(0 << TDC_CONFIG1_START_EDGE_SHIFT)
#define TDC_CONFIG1_START_EDGE_FALLING			(1 << TDC_CONFIG1_START_EDGE_SHIFT)
#define TDC_CONFIG1_MEAS_MODE_SHIFT				1
#define TDC_CONFIG1_MEAS_MODE_1					(0 << TDC_CONFIG1_MEAS_MODE_SHIFT)
#define TDC_CONFIG1_MEAS_MODE_2					(1 << TDC_CONFIG1_MEAS_MODE_SHIFT)
#define TDC_CONFIG1_START_MEAS					1

/*CONFIG2 REG*/
#define TDC_REG_CONFIG2							0x01
#define TDC_CONFIG2_CALIB2_SHIFT				6
#define TDC_CONFIG2_CALIB2_2_CLOCKS				(0 << TDC_CONFIG2_CALIB2_SHIFT)
#define TDC_CONFIG2_CALIB2_10_CLOCKS			(1 << TDC_CONFIG2_CALIB2_SHIFT)
#define TDC_CONFIG2_CALIB2_20_CLOCKS			(2 << TDC_CONFIG2_CALIB2_SHIFT)
#define TDC_CONFIG2_CALIB2_40_CLOCKS			(3 << TDC_CONFIG2_CALIB2_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_SHIFT			3
#define TDC_CONFIG2_AVG_CYCLES_1				(0 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_2				(1 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_4				(2 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_8				(3 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_16				(4 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_32				(5 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_64				(6 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_AVG_CYCLES_128				(7 << TDC_CONFIG2_AVG_CYCLES_SHIFT)
#define TDC_CONFIG2_NUM_STOPS_1					0
#define TDC_CONFIG2_NUM_STOPS_2					1
#define TDC_CONFIG2_NUM_STOPS_3					2
#define TDC_CONFIG2_NUM_STOPS_4					3
#define TDC_CONFIG2_NUM_STOPS_5					4

/*INT_STATUS REG*/
#define TDC_REG_INT_STATUS						0x02
#define TDC_INT_STATUS_MEAS_COMPLETE_FLAG		0x10
#define TDC_INT_STATUS_MEAS_COMPLETE_FLAG_RST	0x10
#define TDC_INT_STATUS_MEAS_STARTED_FLAG		0x08
#define TDC_INT_STATUS_MEAS_STARTED_FLAG_RST	0x08
#define TDC_INT_STATUS_CLK_CNTRL_OVF_INT		0x04
#define TDC_INT_STATUS_CLK_CNTRL_OVF_INT_RST	0x04
#define TDC_INT_STATUS_COARSE_CNTRL_OVF			0x02
#define TDC_INT_STATUS_COARSE_CNTRL_OVF_RST		0x02
#define TDC_INT_STATUS_NEW_MEAS_INT				0x01
#define TDC_INT_STATUS_NEW_MEAS_INT_RST			0x01

/*INT_MASK REG*/
#define TDC_REG_INT_MASK						0x03
#define TDC_INT_MASK_CLK_CNTRL_OVF_INT_DISABLE	0
#define TDC_INT_MASK_CLK_CNTRL_OVF_INT_ENABLED	0x04
#define TDC_INT_MASK_COARSE_CNTR_OVF_INT_DIABLE	0
#define TDC_INT_MASK_COARSE_CNTR_OVF_INT_ENABLE	0x02
#define TDC_INT_MASK_NEW_MEAS_MASK_DISABLE		0
#define TDC_INT_MASK_NEW_MEAS_MASK_ENABLE		0x01

/*COARSE CNTRL OVR HIGH REG*/
#define TDC_REG_COARSE_CNTRL_OVF_H				0x04
#define TDC_COARSE_CNTRL_OVF_H(val)				(val) //Value from 0 - 255

/*COARSE CNTRL OVR LOW REG*/
#define TDC_REG_COARSE_CNTRL_OVF_L				0x05
#define TDC_COARSE_CNTRL_OVF_L(val)				(val) //Value from 0 - 255

/*Clock CNTRL OVR HIGH REG*/
#define TDC_REG_CLOCK_CNTRL_OVF_H				0x06
#define TDC_CLOCK_CNTRL_OVF_H(val)				(val) //Value from 0 - 255

/*Clock CNTRL OVR LOW REG*/
#define TDC_REG_CLOCK_CNTRL_OVF_L				0x07
#define TDC_CLOCK_CNTRL_OVF_L(val)				(val) //Value from 0 - 255

/*Clock CNTRL Stop Mask HIGH REG*/
#define TDC_REG_CLOCK_CNTRL_STOP_MASK_H			0x08
#define TDC_CLOCK_CNTRL_STOP_MASK_H(val)		(val) //Value from 0 - 255

/*Clock CNTRL Stop Mask LOW REG*/
#define TDC_REG_CLOCK_CNTRL_STOP_MASK_L			0x09
#define TDC_CLOCK_CNTRL_STOP_MASK_L(val)		(val) //Value from 0 - 255

/*Time 1 REG*/
#define TDC_REG_TIME1							0x10

/*CLOCK COUNT 1 REG*/
#define TDC_REG_CLOCK1							0x11

/*Time 2 REG*/
#define TDC_REG_TIME2							0x12

/*CLOCK COUNT 2 REG*/
#define TDC_REG_CLOCK2							0x13

/*Time 3 REG*/
#define TDC_REG_TIME3							0x14

/*CLOCK COUNT 3 REG*/
#define TDC_REG_CLOCK3							0x15

/*Time 4 REG*/
#define TDC_REG_TIME4							0x16

/*CLOCK COUNT 4 REG*/
#define TDC_REG_CLOCK4							0x17

/*Time 5 REG*/
#define TDC_REG_TIME5							0x18

/*CLOCK COUNT 5 REG*/
#define TDC_REG_CLOCK5							0x19

/*Time 6 REG*/
#define TDC_REG_TIME6							0x1A

/*CALIBRATION 1 REG*/
#define TDC_REG_CALIBRATION1					0x1B

/*CALIBRATION 2 REG*/
#define TDC_REG_CALIBRATION2					0x1C

#define TDC_TIMER_CLK_PARITY_BIT				(1 << 7)


//=========================================================================
// Prototypes
//=========================================================================
void TDC_Init(SPI_t *spi);
void TDC_StartUltrasonicTransaction(void);
tdcTimerRegs_t *TDC_GetTimerRegs(void);
void TDC_PrintValues(void);

//=========================================================================
// Variables
//=========================================================================


//=========================================================================
// Implementations
//=========================================================================


#endif /* TDC.h */
