/*
 * NextCentury Firmware
 * Copyright (C) 2014 - 2017. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted.
 *
 * Created: 4/21/2017
 * Author: A. Paul
 */


//=========================================================================
// Notes
//=========================================================================


//=========================================================================
// Definitions
//=========================================================================
#define FLOW_TRANSACTION_TIMER	100

//=========================================================================
// Includes
//=========================================================================
#include "sam.h"
#include "TDC.h"
#include "Debug.h"
#include "AFE.h"
#include "SoftTimer.h"

#include "stdlib.h"
#include "stdio.h"

#include "Flow.h"

//=========================================================================
// Types
//=========================================================================
typedef struct RawTimeMeasurment_t
{
	uint32_t	Time1;
	uint32_t	Clock1;
	double		ToF1;
}RawTimeMeasurment_t;

typedef enum FlowDirection_t
{
	UPSTREAM =		0,
	DOWNSTREAM =	1,
}FlowDirection_t;

//=========================================================================
// Prototypes
//=========================================================================
static void flow_setDirection(FlowDirection_t direction);
static void flow_copyTdcValues(void);
static void flow_calculateToFs(void);
static double flow_calcualteToFSingle(double time1, double time2, double clock, double calib);
static void flow_timerCallback(Timer_t *timer);

//=========================================================================
// Variables
//=========================================================================
static SPI_t _spiLine = TI_COMMS_SPI_LINE_CONFIG;
static FlowDirection_t _flowDirection = UPSTREAM;
static volatile tdcTimerRegs_t _rawTimerRegs[2];
static volatile double _rawToFs[2][5];
static volatile double _tofAverage[2];
static volatile double _difference;
static volatile double _avg;
static uint8_t _i = 0;
static Timer_t _timer;

//=========================================================================
// Implementations
//=========================================================================
void FLOW_Init(void)
{
	//Setup Channel Select Pin
	MCU_PinSetup(AFE_CHANNEL_SELECT_PORT, AFE_CHANNEL_SELECT_PIN, 1, _flowDirection, FALSE, FALSE);

	//Initilizes Sensor Libraries
	TDC_Init(&_spiLine);
	AFE_Init(&_spiLine);

	flow_setDirection(UPSTREAM);
	
	TDC_StartUltrasonicTransaction();
	
	/*
	for(uint8_t i = 0; i < 50; i++)
	{
		TDC_PrintValues();
	}
	*/
	
}

void FLOW_TransactionCompleteCallback(TdcStatus_t status)
{
	//Check if error getting reading
	if(status != TDC_SUCCESS)
	{
		DEBUG_WriteLine("ERROR With Reading!");
		return;
	}

	flow_copyTdcValues();

	if(_flowDirection == UPSTREAM)
	{
		flow_setDirection(DOWNSTREAM);
		TDC_StartUltrasonicTransaction();
	}
	else
	{
		flow_calculateToFs();


		/*Calculate Average Flow*/
		_tofAverage[0] = 0;
		_tofAverage[1] = 0;

		for(uint8_t i = 0; i < 5; i++)
			_tofAverage[0] += _rawToFs[0][i];

		for(uint8_t i = 0; i < 5; i++)
			_tofAverage[1] += _rawToFs[1][i];

		_tofAverage[0] = _tofAverage[0] / 5;
		_tofAverage[1] = _tofAverage[1] / 5;

		_difference = _tofAverage[0] - _tofAverage[1];

		//Convert to Nano Seconds
		//_difference = _difference * 1000;
		//_difference = _difference + .010;

		
		//DEBUG_WriteDouble(_tofAverage[0], "us");
		//DEBUG_WriteDouble(_tofAverage[1], "us");

		//_difference += .003;
		//DEBUG_WriteDouble(_difference, "us");
		

		
		if(_i++ < 5)
		{
			_avg += _difference;
		}
		else
		{
			_avg = (_avg / 5);
			DEBUG_WriteDouble(_avg, "us");
			_i = 0;
			_avg = 0;
		}
		

		//Start Timer
		TIMER_Start(&_timer, FLOW_TRANSACTION_TIMER, FALSE, (uint8_t*)&flow_timerCallback);
	}
}

void FLOW_PrintValues(void)
{
	/*
	DEBUG_WriteHex("Timer1 UP: \t", _TimeMeasurementRaw.UpStreamTime1);
	DEBUG_WriteHex("Timer1 DN: \t", _TimeMeasurementRaw.DownStreamTime1);
	DEBUG_WriteHex("Clock1 UP: \t", _TimeMeasurementRaw.UpStreamClock1);
	DEBUG_WriteHex("Clock1 DN: \t", _TimeMeasurementRaw.DownStreamClock1);

	DEBUG_WriteHex("Timer2 UP: \t", _TimeMeasurementRaw.UpStreamTime2);
	DEBUG_WriteHex("Timer2 DN: \t", _TimeMeasurementRaw.DownStreamTime2);
	DEBUG_WriteHex("Clock2 UP: \t", _TimeMeasurementRaw.UpStreamClock2);
	DEBUG_WriteHex("Clock2 DN: \t", _TimeMeasurementRaw.DownStreamClock2);

	DEBUG_WriteHex("Timer3 UP: \t", _TimeMeasurementRaw.UpStreamTime3);
	DEBUG_WriteHex("Timer3 DN: \t", _TimeMeasurementRaw.DownStreamTime3);
	DEBUG_WriteHex("Clock3 UP: \t", _TimeMeasurementRaw.UpStreamClock3);
	DEBUG_WriteHex("Clock3 DN: \t", _TimeMeasurementRaw.DownStreamClock3);

	DEBUG_WriteHex("Timer4 UP: \t", _TimeMeasurementRaw.UpStreamTime4);
	DEBUG_WriteHex("Timer4 DN: \t", _TimeMeasurementRaw.DownStreamTime4);
	DEBUG_WriteHex("Clock4 UP: \t", _TimeMeasurementRaw.UpStreamClock4);
	DEBUG_WriteHex("Clock4 DN: \t", _TimeMeasurementRaw.DownStreamClock4);

	DEBUG_WriteHex("Timer5 UP: \t", _TimeMeasurementRaw.UpStreamTime5);
	DEBUG_WriteHex("Timer5 DN: \t", _TimeMeasurementRaw.DownStreamTime5);
	DEBUG_WriteHex("Clock5 UP: \t", _TimeMeasurementRaw.UpStreamClock5);
	DEBUG_WriteHex("Clock5 DN: \t", _TimeMeasurementRaw.DownStreamClock5);
	*/
}

static void flow_setDirection(FlowDirection_t direction)
{
	if(direction)
		;//DEBUG_WriteLine("Changing Direction to: Downstream");
	else
		;//DEBUG_WriteLine("Changing Direction to: Upstream");

	MCU_SetPinState(AFE_CHANNEL_SELECT_PORT, AFE_CHANNEL_SELECT_PIN, direction);
	_flowDirection = direction;
}

static void flow_copyTdcValues(void)
{
	tdcTimerRegs_t *tdcCpy = TDC_GetTimerRegs();
	tdcTimerRegs_t *localCpy = &_rawTimerRegs[_flowDirection];

	localCpy->Timer1 = tdcCpy->Timer1;
	localCpy->Clock1 = tdcCpy->Clock1;
	localCpy->Timer2 = tdcCpy->Timer2;
	localCpy->Clock2 = tdcCpy->Clock2;
	localCpy->Timer3 = tdcCpy->Timer3;
	localCpy->Clock3 = tdcCpy->Clock3;
	localCpy->Timer4 = tdcCpy->Timer4;
	localCpy->Clock4 = tdcCpy->Clock4;
	localCpy->Timer5 = tdcCpy->Timer5;
	localCpy->Clock5 = tdcCpy->Clock5;
	localCpy->Timer6 = tdcCpy->Timer6;
	localCpy->Calb1 = tdcCpy->Calb1;
	localCpy->Calb2 = tdcCpy->Calb2;
}

static void flow_calculateToFs(void)
{
	for(uint8_t i = 0; i < 2; i++)
	{
		volatile double calibration = (double)(1.125) / ((double)_rawTimerRegs[i].Calb2 - (double)_rawTimerRegs[i].Calb1);
		uint32_t *regs = &_rawTimerRegs[i].Timer1;

		for(uint8_t j = 0; j < 5; j++)
		{
			_rawToFs[i][j] = flow_calcualteToFSingle(regs[0], regs[2], (regs[1] >> 0), calibration);
			*regs += 2;
		}
	}
}

static double flow_calcualteToFSingle(double time1, double time2, double clock, double calib)
{
	volatile double val = (time1 - time2) * calib + (clock * (double).125);
	return val;
}

static void flow_timerCallback(Timer_t *timer)
{
	flow_setDirection(UPSTREAM);
	TDC_StartUltrasonicTransaction();
}
 
 

//=========================================================================
// TaskHandler
//=========================================================================


//=========================================================================
// Interrupt
//=========================================================================

