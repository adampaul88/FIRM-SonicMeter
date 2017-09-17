/*
 * NextCentury Firmware
 * Copyright (C) 2014 - 2017. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted.
 *
 * Created: 4/10/2017
 * Author: A. Paul & K. Paul
 */


//=========================================================================
// Notes
//=========================================================================


//=========================================================================
// Definitions
//=========================================================================
#define AFE_TOTAL_REGISTERS		10

//=========================================================================
// Includes
//=========================================================================
#include "sam.h"
#include "config.h"
#include "Debug.h"
#include "AFE.h"

//=========================================================================
// Types
//=========================================================================
typedef enum AfeStatus_t
{
	Uninitilized,
	Normal,
	ERROR,
}AfeStatus_t;

typedef struct __attribute__((packed)) AFE_t
{
	AfeStatus_t		Status;
	uint8_t			Config0;
	uint8_t			Config1;
	uint8_t			Config2;
	uint8_t			Config3;
	uint8_t			Config4;
	uint8_t			TOF_1;
	uint8_t			TOF_0;
	uint8_t			ErrorFlags;
	uint8_t			Timeout;
	uint8_t			ClockRate;
}AFE_t;

//=========================================================================
// Prototypes
//=========================================================================
static void afe_loadAllValues(void);
static void afe_writeAllValues(void);
static uint8_t afe_getRegister(uint8_t reg);
static void afe_setRegister(uint8_t reg, uint8_t val);
static void afe_setSlaveSelect(uint8_t val);
static void afe_taskHandler(void);

//=========================================================================
// Variables
//=========================================================================
static SPI_t *_spiLine = NULL;
static uint8_t _txInProgress = FALSE;
static AFE_t _afe = { Uninitilized };

//=========================================================================
// Implementations
//=========================================================================
void AFE_Init(SPI_t *spi)
{
	DEBUG_WriteLine("Initializing AFE Library.");

	//Init SPI
	_spiLine = spi;
	SPI_Init(_spiLine);

	//Setup Chip Select Pin
	MCU_PinSetup(SPI_AFE_CS_PORT, SPI_AFE_CS_PIN, 1, 1, FALSE, FALSE);
	//Setup the Enable Pin
	MCU_PinSetup(SPI_AFE_EN_PORT, SPI_AFE_EN_PIN, 1, SPI_AFE_EN_CHIP_ON, FALSE, FALSE);
	//Setup the Rst Pin
	MCU_PinSetup(SPI_AFE_RST_PORT, SPI_AFE_RST_PIN, 1, SPI_AFE_RST_CHIP_ON, FALSE, FALSE);

	/*Setup Defaults*/
	_afe.Config0 = AFE_CONFIG0_TX_FREQ_DIV_8 | AFE_CONFIG0_NUM_TX(5);
	_afe.Config1 = AFE_CONFIG1_NUM_RX_5 | AFE_CONFIG1_NUM_AVG_1_CYCLES;
	_afe.Config2 = AFE_CONFIG2_TOF_MEAS_MODE_2 |  AFE_CONFIG2_EXT_CHSEL_ENABLE;
	_afe.Config3 = AFE_CONFIG3_ECHO_QUAL_THLD_125 | AFE_CONFIG3_BLANKING_ENABLE;
	_afe.Config4 = AFE_CONFIG4_TX_PH_SHIFT_POS(31);
	_afe.TOF_1 = AFE_TOF1_PGA_GAIN_3db;// AFE_TOF1_LNA_CTRL_BYPASS;
	_afe.TOF_0 = AFE_TOF0_TIMING_REG(38);
	_afe.ErrorFlags = 7;
	_afe.Timeout = AFE_TIMEOUT_BLANK_SHORT_PERIOD_128 | AFE_TIMEOUT_TOF_CTRL_512;
	_afe.ClockRate = AFE_CLOCK_RATE_CLK_IN_DIV_1 | AFE_CLOCK_RATE_ATUO_ZERO_64;
	
	//Write default values to AFE Chip
	afe_writeAllValues();
	
	AFE_PrintValues();
}

void AFE_PrintValues(void)
{
	afe_loadAllValues();

	DEBUG_WriteLine("---AFE Values---");

	DEBUG_WriteHex("Config0: ", _afe.Config0);
	DEBUG_WriteHex("Config1: ", _afe.Config1);
	DEBUG_WriteHex("Config2: ", _afe.Config2);
	DEBUG_WriteHex("Config3: ", _afe.Config3);
	DEBUG_WriteHex("Config4: ", _afe.Config4);
	DEBUG_WriteHex("TOF1: ", _afe.TOF_1);
	DEBUG_WriteHex("TOF0: ", _afe.TOF_0);
	DEBUG_WriteHex("Error Flags: ", _afe.ErrorFlags);
	DEBUG_WriteHex("Timeouts: ", _afe.Timeout);
	DEBUG_WriteHex("Clock Rage: ", _afe.ClockRate);
	DEBUG_WriteLine("\r\n\r\n");
}

static void afe_loadAllValues(void)
{
	uint8_t *afeReg = &_afe.Config0;

	for(uint8_t i = 0; i < AFE_TOTAL_REGISTERS; i++)
		afeReg[i] = afe_getRegister(i);

	_afe.Status = Normal;
}

static void afe_writeAllValues(void)
{
	uint8_t *afeReg = &_afe.Config0;

	for(uint8_t i = 0; i < AFE_TOTAL_REGISTERS; i++)
		afe_setRegister(i, afeReg[i]);

	_afe.Status = Normal;
}

static uint8_t afe_getRegister(uint8_t reg)
{
	reg = reg & 0b00111111;

	SPI_TxSendChar(_spiLine, reg, TRUE);
	SPI_TxSendChar(_spiLine, 0x00, FALSE);

	_txInProgress = TRUE;

	//Set Slave Select to 0
	afe_setSlaveSelect(0);

	SPI_TxMasterTransfer(_spiLine);

	while(_txInProgress)
		afe_taskHandler();

	return _spiLine->rxBuffer[1];
}

static void afe_setRegister(uint8_t reg, uint8_t val)
{
	reg = reg & 0b00111111;
	reg |= 0b01000000;

	SPI_TxSendChar(_spiLine, reg, TRUE);
	SPI_TxSendChar(_spiLine, val, FALSE);

	_txInProgress = TRUE;

	//Set Slave Select to 0
	afe_setSlaveSelect(0);

	SPI_TxMasterTransfer(_spiLine);

	while(_txInProgress)
		afe_taskHandler();
}

static void afe_setSlaveSelect(uint8_t val)
{
	MCU_SetPinState(SPI_AFE_CS_PORT, SPI_AFE_CS_PIN, val);
}

//=========================================================================
// TaskHandler
//=========================================================================
static void afe_taskHandler(void)
{
	if(_spiLine->txSize && _spiLine->txIndex >= _spiLine->txSize && _spiLine->rxIndex >= _spiLine->txSize)
	{
		afe_setSlaveSelect(1);
		_spiLine->txSize = 0;
		_txInProgress = FALSE;
	}
}

//=========================================================================
// Interrupt
//=========================================================================

