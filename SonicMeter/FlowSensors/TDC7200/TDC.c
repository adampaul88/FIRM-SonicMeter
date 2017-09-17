/*
 * NextCentury Firmware
 * Copyright (C) 2014 - 2017. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted.
 *
 * Created: 4/11/2017
 * Author: A. Paul & K. Paul
 */


//=========================================================================
// Notes
//=========================================================================


//=========================================================================
// Definitions
//=========================================================================
#define TDC_TOTAL_SMALL_REGISTERS			10
#define TDC_LARGE_REGISTER_START_ADDRESS	0x10
#define TDC_LARGE_REGISTER_END_ADDRESS		0x1C

//=========================================================================
// Includes
//=========================================================================
#include "sam.h"
#include "config.h"
#include "Debug.h"
#include "Input.h"
#include "Flow.h"
#include "AFE.h"
#include "TDC.h"

//=========================================================================
// Types
//=========================================================================
typedef struct __attribute__((packed)) TDC_t
{
	TdcStatus_t		Status;
	uint8_t			Config1;
	uint8_t			Config2;
	uint8_t			IntStatus;
	uint8_t			IntMask;
	uint8_t			CourseCounterHigh;
	uint8_t			CourseCounterLow;
	uint8_t			ClockCounterHigh;
	uint8_t			ClockCounterLow;
	uint8_t			ClockCounterStopMaskHigh;
	uint8_t			ClockCounterStopMaskLow;
	tdcTimerRegs_t	TimerRegs;
}TDC_t;

//=========================================================================
// Prototypes
//=========================================================================
static void tdc_loadAllValues(void);
static void tdc_writeAllValues(void);
static uint32_t tdc_getRegister(uint8_t reg);
static void tdc_setRegister(uint8_t reg, uint8_t val);
static void tdc_setSlaveSelect(uint8_t val);
static void tdc_clearInterrupts(void);
static void tdc_interruptHandler(Input_t *input);
static void tdc_taskHandler(void);

//=========================================================================
// Variables
//=========================================================================
static SPI_t *_spiLine = NULL;
static uint8_t _txInProgress = FALSE;
static TDC_t _tdc = { TDC_UNINITILIZED };
static Input_t _interrupt = TI_TDC_INT_INPUT_CONFIG;

//=========================================================================
// Implementations
//=========================================================================
void TDC_Init(SPI_t *spi)
{
	DEBUG_WriteLine("Initializing TDC Library.");

	//Init SPI
	_spiLine = spi;
	SPI_Init(_spiLine);

	//Setup Chip Select Pin
	MCU_PinSetup(SPI_TDC_CS_PORT, SPI_TDC_CS_PIN, 1, 1, FALSE, FALSE);
	//Setup the Enable Pin
	MCU_PinSetup(SPI_TDC_EN_PORT, SPI_TDC_EN_PIN, 1, SPI_TDC_EN_CHIP_ON, FALSE, FALSE);

	//Setup the Interrupt Pin INput
	_interrupt.CallBackFunc = (uint8_t*)&tdc_interruptHandler;
	INPUT_Init(&_interrupt);
	
	/*Setup Defaults*/
	_tdc.Config1 = TDC_CONFIG1_MEAS_MODE_2;
	_tdc.Config2 = TDC_CONFIG2_CALIB2_10_CLOCKS | TDC_CONFIG2_NUM_STOPS_5 | TDC_CONFIG2_AVG_CYCLES_1;
	_tdc.IntStatus =  0x1F;
	_tdc.IntMask = TDC_INT_MASK_CLK_CNTRL_OVF_INT_ENABLED | TDC_INT_MASK_COARSE_CNTR_OVF_INT_ENABLE | TDC_INT_MASK_NEW_MEAS_MASK_ENABLE;
	_tdc.CourseCounterHigh = TDC_COARSE_CNTRL_OVF_H(0xFF);
	_tdc.CourseCounterLow = TDC_COARSE_CNTRL_OVF_L(0xFF);
	_tdc.ClockCounterHigh = TDC_CLOCK_CNTRL_OVF_H(0xFF);
	_tdc.ClockCounterLow = TDC_CLOCK_CNTRL_OVF_L(0xFF);
	_tdc.ClockCounterStopMaskHigh = 0;
	_tdc.ClockCounterStopMaskLow = 0;

	_tdc.Status = TDC_NORMAL;

	tdc_writeAllValues();

	INPUT_TaskHandler();
	
	TDC_PrintValues();
}

void TDC_StartUltrasonicTransaction(void)
{
	//DEBUG_WriteLine("Starting Ultrasonic Transaction \r\n");

	tdc_clearInterrupts();
	
	_tdc.Status = TDC_NORMAL;
	tdc_setRegister(TDC_REG_CONFIG1, TDC_CONFIG1_START_MEAS | TDC_CONFIG1_MEAS_MODE_2);
	
}

tdcTimerRegs_t *TDC_GetTimerRegs(void)
{
	return &_tdc.TimerRegs;
}

void TDC_PrintValues(void)
{
	tdc_loadAllValues();

	DEBUG_WriteLine("---TDC Values---");

	DEBUG_WriteHex("Config1: ", _tdc.Config1);
	DEBUG_WriteHex("Config2: ", _tdc.Config2);
	DEBUG_WriteHex("IntStatus: ", _tdc.IntStatus);
	DEBUG_WriteHex("IntMask: ", _tdc.IntMask);
	DEBUG_WriteHex("CourseCounterH: ", _tdc.CourseCounterHigh);
	DEBUG_WriteHex("CourseCounterL: ", _tdc.CourseCounterLow);
	DEBUG_WriteHex("ClockCounterL: ", _tdc.ClockCounterHigh);
	DEBUG_WriteHex("ClockCounterH: ", _tdc.ClockCounterLow);
	DEBUG_WriteHex("ClockCounterMaskH: ", _tdc.ClockCounterStopMaskHigh);
	DEBUG_WriteHex("ClockCounterMaskL: ", _tdc.ClockCounterStopMaskLow);
	DEBUG_WriteHex("Timer1: ", _tdc.TimerRegs.Timer1);
	DEBUG_WriteHex("Clock1: ", _tdc.TimerRegs.Clock1);
	DEBUG_WriteHex("Timer2: ", _tdc.TimerRegs.Timer2);
	DEBUG_WriteHex("Clock2: ", _tdc.TimerRegs.Clock2);
	DEBUG_WriteHex("Timer3: ", _tdc.TimerRegs.Timer3);
	DEBUG_WriteHex("Clock3: ", _tdc.TimerRegs.Clock3);
	DEBUG_WriteHex("Timer4: ", _tdc.TimerRegs.Timer4);
	DEBUG_WriteHex("Clock4: ", _tdc.TimerRegs.Clock4);
	DEBUG_WriteHex("Timer5: ", _tdc.TimerRegs.Timer5);
	DEBUG_WriteHex("Clock5: ", _tdc.TimerRegs.Clock5);
	DEBUG_WriteHex("Timer6: ", _tdc.TimerRegs.Timer6);
	DEBUG_WriteHex("Calb1: ", _tdc.TimerRegs.Calb1);
	DEBUG_WriteHex("Calb2: ", _tdc.TimerRegs.Calb2);
	
	DEBUG_WriteLine("\r\n\r\n");
}

static void tdc_loadAllValues(void)
{
	uint8_t *tdcReg = &_tdc.Config1;

	uint8_t regLocation = 0;

	//Load Setting Registers
	for(; regLocation < TDC_TOTAL_SMALL_REGISTERS; regLocation++)
		tdcReg[regLocation] = tdc_getRegister(regLocation);

	//Load Timer Registers
	_tdc.TimerRegs.Timer1 = tdc_getRegister(TDC_REG_TIME1);
	_tdc.TimerRegs.Clock1 = tdc_getRegister(TDC_REG_CLOCK1);
	_tdc.TimerRegs.Timer2 = tdc_getRegister(TDC_REG_TIME2);
	_tdc.TimerRegs.Clock2 = tdc_getRegister(TDC_REG_CLOCK2);
	_tdc.TimerRegs.Timer3 = tdc_getRegister(TDC_REG_TIME3);
	_tdc.TimerRegs.Clock3 = tdc_getRegister(TDC_REG_CLOCK3);
	_tdc.TimerRegs.Timer4 = tdc_getRegister(TDC_REG_TIME4);
	_tdc.TimerRegs.Clock4 = tdc_getRegister(TDC_REG_CLOCK4);
	_tdc.TimerRegs.Timer5 = tdc_getRegister(TDC_REG_TIME5);
	_tdc.TimerRegs.Clock5 = tdc_getRegister(TDC_REG_CLOCK5);
	_tdc.TimerRegs.Timer6 = tdc_getRegister(TDC_REG_TIME6);
	_tdc.TimerRegs.Calb1 = tdc_getRegister(TDC_REG_CALIBRATION1);
	_tdc.TimerRegs.Calb2 = tdc_getRegister(TDC_REG_CALIBRATION2);
}

static void tdc_writeAllValues(void)
{
	//Writes all registers (Not including timer and clock registers)
	uint8_t *tdcReg = &_tdc.Config1;

	for(uint8_t i = 0; i < TDC_TOTAL_SMALL_REGISTERS; i++)
		tdc_setRegister(i, tdcReg[i]);
}


static uint32_t tdc_getRegister(uint8_t reg)
{
	reg = reg & 0b00111111;

	SPI_TxSendChar(_spiLine, reg, TRUE);
	SPI_TxSendChar(_spiLine, 0x00, FALSE);

	if(reg >= TDC_LARGE_REGISTER_START_ADDRESS)
	{
		SPI_TxSendChar(_spiLine, 0x00, FALSE);
		SPI_TxSendChar(_spiLine, 0x00, FALSE);
	}

	_txInProgress = TRUE;

	//Set Slave Select to 0
	tdc_setSlaveSelect(0);

	SPI_TxMasterTransfer(_spiLine);

	while(_txInProgress)
		tdc_taskHandler();

	if(reg >= TDC_LARGE_REGISTER_START_ADDRESS)
		return (uint32_t)( (_spiLine->rxBuffer[1] << 16) | (_spiLine->rxBuffer[2] << 8) | _spiLine->rxBuffer[3] );
	else
		return _spiLine->rxBuffer[1];
}

static void tdc_setRegister(uint8_t reg, uint8_t val)
{
	reg = reg & 0b00111111;
	reg |= 0b01000000;

	SPI_TxSendChar(_spiLine, reg, TRUE);
	SPI_TxSendChar(_spiLine, val, FALSE);

	_txInProgress = TRUE;

	//Set Slave Select to 0
	tdc_setSlaveSelect(0);

	SPI_TxMasterTransfer(_spiLine);

	while(_txInProgress)
		tdc_taskHandler();
}

static void tdc_clearInterrupts(void)
{
	tdc_setRegister(TDC_REG_INT_STATUS, 0x1F);
	INPUT_TaskHandler();
}

static void tdc_setSlaveSelect(uint8_t val)
{
	MCU_SetPinState(SPI_TDC_CS_PORT, SPI_TDC_CS_PIN, val);
}

static void tdc_interruptHandler(Input_t *input)
{
	//DEBUG_WriteNumber("Interrupt Pin State is: ", input->State);

	if(!input->State)
	{
		tdc_loadAllValues();
		tdc_clearInterrupts();
		
		if(_tdc.IntStatus != 0x19)
			_tdc.Status = TDC_ERROR;
		else
			_tdc.Status = TDC_SUCCESS;

		FLOW_TransactionCompleteCallback(_tdc.Status);
	}
}

//=========================================================================
// TaskHandler
//=========================================================================
static void tdc_taskHandler(void)
{
	if(_spiLine->txSize && _spiLine->txIndex >= _spiLine->txSize && _spiLine->rxIndex >= _spiLine->txSize)
	{
		tdc_setSlaveSelect(1);
		_spiLine->txSize = 0;
		_txInProgress = FALSE;
	}
}

//=========================================================================
// Interrupt
//=========================================================================

