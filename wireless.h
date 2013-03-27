/**
*@file wireless.h
*@author Dirk Dubois, Alain Slak
*@date March 20th, 2013
*@brief A set of functions to interface with the ez430-CC2500
*
*/

#ifndef __WIRELESS_H
#define __WIRELESS_H

//defines
#define SMARTRF_SETTING_FSCTRL1 0x0C
#define SMARTRF_SETTING_FSCTRL0 0x00
#define SMARTRF_SETTING_FREQ2 0x5D
#define SMARTRF_SETTING_FREQ1 0x93
#define SMARTRF_SETTING_FREQ0 0xB1
#define SMARTRF_SETTING_MDMCFG4 0x0E
#define SMARTRF_SETTING_MDMCFG3 0x3B
#define SMARTRF_SETTING_MDMCFG2 0x73
#define SMARTRF_SETTING_MDMCFG1 0x42
#define SMARTRF_SETTING_MDMCFG0 0xF8
#define SMARTRF_SETTING_CHANNR 0x00
#define SMARTRF_SETTING_DEVIATN 0x00
#define SMARTRF_SETTING_FREND1 0xB6
#define SMARTRF_SETTING_FREND0 0x10
#define SMARTRF_SETTING_MCSM0 0x18
#define SMARTRF_SETTING_FOCCFG 0x1D
#define SMARTRF_SETTING_BSCFG 0x1C
#define SMARTRF_SETTING_AGCCTRL2 0xC7
#define SMARTRF_SETTING_AGCCTRL1 0x40
#define SMARTRF_SETTING_AGCCTRL0 0xB0
#define SMARTRF_SETTING_FSCAL3 0xEA
#define SMARTRF_SETTING_FSCAL2 0x0A
#define SMARTRF_SETTING_FSCAL1 0x00
#define SMARTRF_SETTING_FSCAL0 0x19
#define SMARTRF_SETTING_FSTEST 0x59
#define SMARTRF_SETTING_TEST2 0x88
#define SMARTRF_SETTING_TEST1 0x31
#define SMARTRF_SETTING_TEST0 0x0B
#define SMARTRF_SETTING_FIFOTHR 0x07
#define SMARTRF_SETTING_IOCFG2 0x29
#define SMARTRF_SETTING_IOCFG0D 0x06
#define SMARTRF_SETTING_PKTCTRL1 0x04
#define SMARTRF_SETTING_PKTCTRL0 0x05
#define SMARTRF_SETTING_ADDR 0x00
#define SMARTRF_SETTING_PKTLEN 0x0A

#define PARTNUM 0x30

/*Command strobes*/
#define SRX 0x34

/*Wireless SPI defines*/
#define WIRELESS_BUFFER_SIZE 7
#define WIRELESS_CS_PIN 0x0010 /**<Select pin4 for wireless SPI1 CSn*/
#define WIRELESS_CS_PORT GPIOA /**<Select portA for wireless SPI CSn*/

/*Wireless Command defines*/
#define SINGLEBYTE_WR 0x00
#define MULTIPLEBYTE_WR 0x40
#define SINGLEBYTE_RD 0x80
#define MULTIPLEBYTE_RD 0xC0


/**
*@brief A function to setup and initialize wireless communication
*@param[inout] None
*@retval None
*/
void initWireless(void);

/**
*@brief A function to write to a specified register on the wireless board
*@param[in] data The data to be sent to the device
*@param[in] address The address of the register to send data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval None
*/
void wirelessWrite(uint8_t* data, uint8_t address, uint16_t numOfBytes);

/**
*@brief A function that writes a byte for the Wireless module over SPI
*@param[in] data The data to be written to the wireless device
*@retval None
*/
void wirelessSendByte(uint8_t data);

/**
*@brief A function to read from a specified resgister on the wireless board
*@param[inout] data The data to be read to the device
*@param[in] address The address of the register to read data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval None
*/
void wirelessRead(uint8_t* data, uint8_t address, uint16_t numOfBytes);

#endif
