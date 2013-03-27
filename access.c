/**
*@file access.c
*@author Dirk Dubois, Alain Slak
*@date March 6th, 2013
*@brief A set of functions that provide access to global variables shared accross multiple threads
*
*/

/*Includes*/
#include "access.h"
#include "common.h"
#include "cmsis_os.h"
#include <stdint.h>
/**
*@brief A function that safely access's the corrected values of the accelerometer
*@param[inout] accValues A pointer to the new location in memory that data is copied to
*@retval None
*/
void getACCValues(float* accValues)
{
	
	int i = 0;
	for(i = 0; i < 3; i++){
		osSemaphoreWait(accId, osWaitForever);
		accValues[i] = accCorrectedValues[i]; //Critical access portion
		osSemaphoreRelease(accId);
	}
}

/**
*@brief A function that safely access's the other board's accelerometer readings
*@param[inout] accValues A pointer to the new location in memory that data is copied to
*@retval None
*/
void getWirelessACCValues(float* accValues)
{
	
	int i = 0;
	for(i = 0; i < 3; i++){
		osSemaphoreWait(wirelessAccId, osWaitForever);
		accValues[i] = accCorrectedValues[i]; //Critical access portion
		osSemaphoreRelease(wirelessAccId);
	}
}

/**
*@brief A function that gets the information recieved over wireless to be processed
*@param[inout] rxBuffer is the buffer to which the data is copied to
*@param[in] bufferSize The size of the passed in buffer
*@warning bufferSize must be of the same size as WIRELESS_BUFFER_SIZE, see wireless.h for details
*@retval None
*/
void getRecieved(uint8_t* rxBuffer, uint8_t bufferSize){
	
	uint8_t i = 0;
	for( i =0; i< bufferSize; i++){		
		osSemaphoreWait(rxId, osWaitForever);
		rxBuffer[i] = rxWireless[i]; //Critical access portion
		osSemaphoreRelease(rxId);
	}
}
/**
*@brief A function that sets the information to be transmitted over wireless
*@param[inout] txBuffer is the buffer from which the data is copied from
*@retval None
*/
void setTransmit(uint8_t* txBuffer);

