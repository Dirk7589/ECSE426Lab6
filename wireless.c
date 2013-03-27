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

	GPIO_ResetBits(WIRELESS_CS_PORT, (uint16_t)WIRELESS_CS_PIN); //Lower CS line
	//stream0 is rx, stream3 is tx

	//DMA2_Stream0->M0AR = (uint32_t)rxptr;
	DMA2_Stream0->NDTR = 48;
	DMA2_Stream0->M0AR = (uint32_t)rxWirelessInit;

	//DMA2_Stream3->M0AR = (uint32_t)txptr;
	DMA2_Stream3->NDTR = 48;
	DMA2_Stream3->M0AR = (uint32_t)txWirelessInit;
	
		//Enable DMA
	DMA_Cmd(DMA2_Stream0, ENABLE); // RX
	DMA_Cmd(DMA2_Stream3, ENABLE); // TX
	
	dmaInitFlag = 1;
	
	while(dmaInitFlag);
	DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, DISABLE); //Disable the corresponding NVIC interupt
	//SPI_DMA_Transfer(rxWirelessInit, txWirelessInit, 48, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
	
	//osSignalWait(dmaFlag, osWaitForever);
	//osMutexRelease(dmaId);//Clear Mutex	
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

