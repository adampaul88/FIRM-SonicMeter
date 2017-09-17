/*
 * NextCentury Firmware
 * Copyright (C) 2014 - 2017. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted.
 *
 * Created: 4/10/2017
 *  Author: A. Paul & K. Paul
 */


#ifndef AFE_H_
#define AFE_H_

//=========================================================================
// Notes
//=========================================================================


//=========================================================================
// Definitions
//=========================================================================

/*CONFIG 0 Reg*/
#define AFE_REG_CONFIG0						0x00
#define AFE_CONFIG0_TX_FREQ_DIV_SHIFT		5
#define AFE_CONFIG0_TX_FREQ_DIV_2			(0 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_4			(1 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_8			(2 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_16			(3 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_32			(4 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_64			(5 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_128			(6 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_TX_FREQ_DIV_256			(7 << AFE_CONFIG0_TX_FREQ_DIV_SHIFT)
#define AFE_CONFIG0_NUM_TX(pulses)			(pulses << 0) //Tx Pulses from 1 - 31

/*CONFIG 1 Reg*/
#define AFE_REG_CONFIG1						0x01
#define AFE_CONFIG1_NUM_AVG_SHIFT			3
#define AFE_CONFIG1_NUM_AVG_1_CYCLES		(0 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_2_CYCLES		(1 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_4_CYCLES		(2 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_8_CYCLES		(3 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_16_CYCLES		(4 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_32_CYCLES		(5 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_64_CYCLES		(6 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_AVG_128_CYCLES		(7 << AFE_CONFIG1_NUM_AVG_SHIFT)
#define AFE_CONFIG1_NUM_RX_SHIFT			0
#define AFE_CONFIG1_NUM_RX_32				(0 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_1				(1 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_2				(2 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_3				(3 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_4				(4 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_5				(5 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_6				(6 << AFE_CONFIG1_NUM_RX_SHIFT)
#define AFE_CONFIG1_NUM_RX_7				(7 << AFE_CONFIG1_NUM_RX_SHIFT)

/*CONFIG 2 Reg*/
#define AFE_REG_CONFIG2						0x02
#define AFE_CONFIG2_VCOM_SEL_EXTERNAL		0x80
#define AFE_CONFIG2_VCOM_SEL_INTERNAL		0
#define AFE_CONFIG2_MEAS_MODE_TOF			0
#define AFE_CONFIG2_MEAS_MODE_TEMP			0x40
#define AFE_CONFIG2_DAMPING_DISABLE			0
#define AFE_CONFIG2_DAMPING_ENABLE			0x20
#define AFE_CONFIG2_CH_SWAP_AUTO_DISABLE	0
#define AFE_CONFIG2_CH_SWAP_AUTO_ENABLE		0x10
#define AFE_CONFIG2_EXT_CHSEL_DISABLE		0
#define AFE_CONFIG2_EXT_CHSEL_ENABLE		0x08
#define AFE_CONFIG2_CH_SEL_0				0
#define AFE_CONFIG2_CH_SEL_1				0x04
#define AFE_CONFIG2_TOF_MEAS_MODE_0			0
#define AFE_CONFIG2_TOF_MEAS_MODE_1			0x01
#define AFE_CONFIG2_TOF_MEAS_MODE_2			0x02

/*CONFIG 3 Reg*/
#define AFE_REG_CONFIG3						0x03
#define AFE_CONFIG3_TEMP_MODE_REF_RTD1_RTD2	0
#define AFE_CONFIG3_TEMP_MODE_REF_RTD1		0x40
#define AFE_CONFIG3_TEMP_RTD_SEL_PT1000		0
#define AFE_CONFIG3_TEMP_RTD_SEL_PT500		0x20
#define AFE_CONFIG3_TEMP_CLK_DIV_8			0
#define AFE_CONFIG3_TEMP_CLK_DIV_TX_FREQ	0x10
#define AFE_CONFIG3_BLANKING_DISABLE		0
#define AFE_CONFIG3_BLANKING_ENABLE			0x08
#define AFE_CONFIG3_ECHO_QUAL_THLD_35		0
#define AFE_CONFIG3_ECHO_QUAL_THLD_50		1
#define AFE_CONFIG3_ECHO_QUAL_THLD_75		2
#define AFE_CONFIG3_ECHO_QUAL_THLD_125		3
#define AFE_CONFIG3_ECHO_QUAL_THLD_220		4
#define AFE_CONFIG3_ECHO_QUAL_THLD_410		5
#define AFE_CONFIG3_ECHO_QUAL_THLD_775		6
#define AFE_CONFIG3_ECHO_QUAL_THLD_1500		7

/*CONFIG 4 REG*/
#define AFE_REG_CONFIG4						0x04
#define AFE_CONFIG4_RECEIVE_MODE_SINGLE		0
#define AFE_CONFIG4_RECEIVE_MODE_MULTI		0x40
#define AFE_CONFIG4_TRIG_EDGE_POL_RISING	0
#define AFE_CONFIG4_TRIG_EDGE_POL_FALLING	0x20
#define AFE_CONFIG4_TX_PH_SHIFT_POS(val)	(val & 0b00011111) //From 2- 31

/*TOF 1 REG*/
#define AFE_REG_TOF1						0x5
#define AFE_TOF1_PGA_GAIN_SHIFT				5
#define AFE_TOF1_PGA_GAIN_0db				(0 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_3db				(1 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_6db				(2 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_9db				(3 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_12db				(4 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_15db				(5 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_18db				(6 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_GAIN_21db				(7 << AFE_TOF1_PGA_GAIN_SHIFT)
#define AFE_TOF1_PGA_CTRL_ACTIVE			0
#define AFE_TOF1_PGA_CTRL_BYPASS			0x10
#define AFE_TOF1_LNA_CTRL_ACTIVE			0
#define AFE_TOF1_LNA_CTRL_BYPASS			0x08
#define AFE_TOF1_LNA_FB_CAPACITIVE			0
#define AFE_TOF1_LNA_FB_RESISTIVE			0x04
#define AFE_TOF1_TIMING_REG(val)			(val & 0b00000011)

/*TOF 0 REG*/
#define AFE_REG_TOF0						0x06
#define AFE_TOF0_TIMING_REG(val)			val

/*ERROR Flags Reg*/
#define AFE_REG_ERROR_FLAGS					0x07
#define AFE_ERROR_FLAGS_SIGNAL_WEAK			0x04
#define AFE_ERROR_FLAGS_NO_SIGNAL			0x02
#define AFE_ERROR_FLAGS_NO_SIGNAL_RST		0x02
#define AFE_ERROR_FLAGS_SIGNAL_HIGH			0x01
#define AFE_ERROR_FLAGS_SIGNAL_HIGH_RST		0x01

/*TIMEOUT REG*/
#define AFE_REG_TIMEOUT							0x08
#define AFE_TIMEOUT_FORCE_SHORT_TOF_DISABLE		0
#define AFE_TIMEOUT_FORCE_SHORT_TOF				0x40
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT	3
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_8		(0 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_16		(1 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_32		(2 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_64		(3 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_128		(4 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_256		(5 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_512		(6 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_BLANK_SHORT_PERIOD_1024		(7 << AFE_TIMEOUT_BLANK_SHORT_PERIOD_SHIFT)
#define AFE_TIMEOUT_ECHO_ENABLE					0
#define AFE_TIMEOUT_ECHO_DISABLE				0x04
#define AFE_TIMEOUT_TOF_CTRL_128				0
#define AFE_TIMEOUT_TOF_CTRL_256				1
#define AFE_TIMEOUT_TOF_CTRL_512				2
#define AFE_TIMEOUT_TOF_CTRL_1024				3

/*Clock Rate REG*/
#define AFE_REG_CLOCK_RATE						0x09
#define AFE_CLOCK_RATE_CLK_IN_DIV_1				0
#define AFE_CLOCK_RATE_CLK_IN_DIV_2				0x04
#define AFE_CLOCK_RATE_ATUO_ZERO_64				0
#define AFE_CLOCK_RATE_ATUO_ZERO_128			1
#define AFE_CLOCK_RATE_ATUO_ZERO_256			2
#define AFE_CLOCK_RATE_ATUO_ZERO_512			3




//=========================================================================
// Includes
//=========================================================================
#include "Spi.h"

//=========================================================================
// Types
//=========================================================================


//=========================================================================
// Prototypes
//=========================================================================
void AFE_Init(SPI_t *spi);
void AFE_PrintValues(void);

//=========================================================================
// Variables
//=========================================================================


//=========================================================================
// Implementations
//=========================================================================


#endif /* AFE.h */
