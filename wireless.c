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
	
	uint8_t txWirelessInit[48] = {0x00|MULTIPLEBYTE_WR, SMARTRF_SETTING_IOCFG2, SMARTRF_SETTING_IOCFG1,
	SMARTRF_SETTING_IOCFG0, SMARTRF_SETTING_FIFOTHR, SMARTRF_SETTING_SYNC1, SMARTRF_SETTING_SYNC0,
	SMARTRF_SETTING_PKTLEN,SMARTRF_SETTING_PKTCTRL1, SMARTRF_SETTING_PKTCTRL0, SMARTRF_SETTING_ADDR,
	SMARTRF_SETTING_CHANNR, SMARTRF_SETTING_FSCTRL1, SMARTRF_SETTING_FSCTRL0, SMARTRF_SETTING_FREQ2,
	SMARTRF_SETTING_FREQ1, SMARTRF_SETTING_FREQ0, SMARTRF_SETTING_MDMCFG4, SMARTRF_SETTING_MDMCFG3,
	SMARTRF_SETTING_MDMCFG2,SMARTRF_SETTING_MDMCFG1, SMARTRF_SETTING_MDMCFG0, SMARTRF_SETTING_DEVIATN,
	SMARTRF_SETTING_MCSM2,SMARTRF_SETTING_MCSM1, SMARTRF_SETTING_MCSM0,SMARTRF_SETTING_FOCCFG, SMARTRF_SETTING_BSCFG,
  SMARTRF_SETTING_AGCCTRL2, SMARTRF_SETTING_AGCCTRL1, SMARTRF_SETTING_AGCCTRL0, SMARTRF_SETTING_WOREVT1, 
	SMARTRF_SETTING_WOREVT0,SMARTRF_SETTING_WORCTRL, SMARTRF_SETTING_FREND1,SMARTRF_SETTING_FREND0, SMARTRF_SETTING_FSCAL3,
  SMARTRF_SETTING_FSCAL2,SMARTRF_SETTING_FSCAL1, SMARTRF_SETTING_FSCAL0, SMARTRF_SETTING_RCCTRL1, SMARTRF_SETTING_RCCTRL0,
	SMARTRF_SETTING_FSTEST, SMARTRF_SETTING_PTEST, SMARTRF_SETTING_AGCTEST, SMARTRF_SETTING_TEST2, SMARTRF_SETTING_TEST1, 
	SMARTRF_SETTING_TEST0}; /**<Transmission buffer for Wireless for DMA*/
	
	uint8_t rxWirelessInit[48]; /**<Receive buffer for Wireless for DMA*/

	SPI_DMA_Transfer(rxWirelessInit, txWirelessInit, 48, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
	
	osSignalWait(dmaFlag, osWaitForever);
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
	

  if(numOfBytes > 0x01)
  {
    address |= (uint8_t)MULTIPLEBYTE_WR;
  }
  /* Set chip select Low at the start of the transmission */
  GPIO_ResetBits(WIRELESS_CS_PORT, (uint16_t)WIRELESS_CS_PIN); //Lower CS line
  
  /* Send the Address of the indexed register */
  wirelessSendByte(address);
  /* Send the data that will be written into the device (MSB First) */
  while(numOfBytes >= 0x01)
  {
    wirelessSendByte(*data);
    numOfBytes--;
    data++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GPIO_SetBits(WIRELESS_CS_PORT, (uint16_t)WIRELESS_CS_PIN); //Lower CS line
}

/**
*@brief A function that writes a byte for the Wireless module over SPI
*@param[in] data The data to be written to the wireless device
*@retval None
*/
void wirelessSendByte(uint8_t data){
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //Wait for previous transfer to complete
	SPI1->DR = data; //Send byte
}

/**
*@brief A function to read from a specified resgister on the wireless board
*@param[inout] data The data to be read to the device
*@param[in] address The address of the register to read data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval The received value
*/
void wirelessRead(uint8_t* data, uint8_t address, uint16_t numOfBytes){
	
	uint8_t i;
	uint8_t receivedValue = 0;
	uint8_t transmittedValue[numOfBytes];

	for(i = 0; i < numOfBytes; i++)
	{
		transmittedValue[i] = 0;
	}
	transmittedValue[0] = address;
	
	if(numOfBytes > 2)
	{
    address |= (uint8_t)(MULTIPLEBYTE_RD);
  }
  else
  {
    address |= (uint8_t)(SINGLEBYTE_RD);
  }

	SPI_DMA_Transfer(data, &transmittedValue[0], numOfBytes, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
}

