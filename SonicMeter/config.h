/*
 * config.h
 *
 * Created: 1/17/17 7:41:17 AM
 *  Author: adampaul
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

//=========================================================================
// Configurations
//=========================================================================


//=========================================================================
// Version
//=========================================================================
#define FIRMWARE_VERSION_DEVICE_TYPE		0xFE //Unit Test Device Type
#define FIRMWARE_VERSION_MAJOR				1
#define FIRMWARE_VERSION_MINOR				7


//=========================================================================
// Flash Information
//=========================================================================
#define FLASH_PROGRAM_SECTOR_PAGES			984

#define SOFT_EEPROM_ADDRESS					0x1F000
#define SOFT_EEPROM_SECTOR_SIZE_BYTES		32
#define SOFT_EEPROM_PAGES					32

#define UPDTAE_BOOTLOADER_FLAGS_ADDRESS		0x1F800
#define UPDATE_BOOTLOADER_FLAGS_PAGES		16
#define UPDATE_BOOTLOADER_FUNCS_ADDRESS		0x1FC00
#define UPDATE_BOOTLOADER_FUNCS_PAGES		16
#define SPI_BUFFER_SIZE						250

//=========================================================================
// PINS
//=========================================================================
//AFE 1000 Chip Select PIn
#define SPI_AFE_CS_PORT			PORT_A
#define SPI_AFE_CS_PIN			4

//AFE 1000 Enable Pin
#define SPI_AFE_EN_PORT			PORT_B
#define SPI_AFE_EN_PIN			30
#define SPI_AFE_EN_CHIP_ON		1
#define SPI_AFE_EN_CHIP_OFF		0

//AFE 1000 Channel Select Pin
#define AFE_CHANNEL_SELECT_PORT	PORT_A
#define AFE_CHANNEL_SELECT_PIN	2

//AFE 1000 RST Pin
#define SPI_AFE_RST_PORT		PORT_C
#define SPI_AFE_RST_PIN			2
#define SPI_AFE_RST_CHIP_ON		0
#define SPI_AFE_RST_CHIP_OFF	1

//TDC 7200 Chip Select Pin
#define SPI_TDC_CS_PORT			PORT_B
#define SPI_TDC_CS_PIN			4

//TDC 7200 Enable Pin
#define SPI_TDC_EN_PORT			PORT_B
#define SPI_TDC_EN_PIN			8
#define SPI_TDC_EN_CHIP_ON		1
#define SPI_TDC_EN_CHIP_OFF		0


//=========================================================================
// Inline Functions
//=========================================================================
#define SERCOM_CS_MOD \
	if(flags & SERCOM_SPI_INTFLAG_TXC)\
	{\
		if(*port->TxIndex == *port->TxSize)\
			PORT->Group[SPI_TDC_CS_PORT].OUTSET.reg = (1 << SPI_TDC_CS_PIN);\
	}\


//=========================================================================
// Instance Defaults
//=========================================================================
//Inturrupt Input Button Config
#define TI_TDC_INT_INPUT_CONFIG { \
	FALSE, \
	0, \
	PORT_A, \
	20, \
	0, \
0, }\

#define TI_COMMS_SPI_LINE_CONFIG { \
	FALSE, \
	FALSE, \
	PORT_B, \
	2, \
	PORT_B, \
	0, \
	PORT_B, \
	1, \
	PORT_B, \
	21, \
	0, \
	1, \
	PORT_PMUX_PMUXE(2), \
COMM3, }\


//Debug uart pin for SAML22 Eval Board
#define DEBUG_UART_LINE_CONIF { \
	FALSE, \
	FALSE, \
	PORT_C, \
	24, \
	PORT_C, \
	25, \
	1, \
	3, \
	115200, \
	LSBF, \
	COMM4, \
PORT_PMUX_PMUXE(3) }

#endif /* CONFIG_H_ */