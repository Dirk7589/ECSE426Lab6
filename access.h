/**
*@file access.h
*@author Dirk Dubois, Alain Slak
*@date March 6th, 2013
*@brief A set of functions that provide access to global variables shared accross multiple threads
*/

#ifndef __ACCESS_H
#define __ACCESS_H

/**
*@brief A function that safely access's the corrected values of the accelerometer
*@param[inout] accValues A pointer to the new location in memory that data is copied to
*@retval None
*/
void getACCValues(float* accValues);

/**
*@brief A function that gets the information recieved over wireless to be processed
*@param[inout] rxBuffer is the buffer to which the data is stored
*@retval None
*/
void getRecieved(uint8_t* rxBuffer);

/**
*@brief A function that sets the information to be transmitted over wireless
*@param[inout] txBuffer is the buffer from which the data is transferred
*@retval None
*/
void setTransmit(uint8_t* txBuffer);
#endif
