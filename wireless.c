/**
*@file wireless.c
*@author Dirk Dubois, Alain Slak
*@date March 20th, 2013
*@brief A set of functions to interface with the ez430-CC2500
*
*/

/*Includes*/
#include <stdint.h>
#include "stm32f4xx.h"
#include "spi.h"
#include "wireless.h"
#include "common.h"


/**
*@brief A function to setup and initialize wireless communication
*@param[inout] None
*@retval None
*/
void initWireless(void){
	
	strobeCommand[0] = IDLE|SINGLEBYTE_WR; //Set for receive mode
	SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
	while(dmaFromWirelessFlag);
	osMutexRelease(dmaId);
	
	SPI_DMA_Transfer(rxWirelessInit, txWirelessInit, WIRELESS_BUFFER_INIT_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
	//Wait for the wireless INIT to finish
 	while(dmaFromWirelessFlag);
 	osMutexRelease(dmaId);//Clear Mutex
	
	txWireless[0] = 0x00 | MULTIPLEBYTE_RD;
	SPI_DMA_Transfer(rxWireless, txWireless, WIRELESS_BUFFER_INIT_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
	//Wait for the wireless INIT to finish
 	while(dmaFromWirelessFlag);
 	osMutexRelease(dmaId);//Clear Mutex
}

/**
*@brief A function to write to a specified register on the wireless board
*@param[in] data The data to be sent to the device
*@param[in] address The address of the register to send data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval None
*/
void wirelessWrite(uint8_t* data, uint8_t address, uint16_t numOfBytes){

}

/**
*@brief A function that writes a byte for the Wireless module over SPI
*@param[in] data The data to be written to the wireless device
*@retval None
*/
void wirelessSendByte(uint8_t data){

}

/**
*@brief A function to read from a specified resgister on the wireless board
*@param[inout] data The data to be read to the device
*@param[in] address The address of the register to read data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval The received value
*/
void wirelessRead(uint8_t* data, uint8_t address, uint16_t numOfBytes){

}

