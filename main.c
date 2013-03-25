 /**
*@file main.c
*@author Dirk Dubois, Alain Slak
*@date February 21th, 2013
*@brief 
*
*/

/*Includes*/
#include <stdint.h>
#include <arm_math.h>
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "init.h"
#include "initACC.h"
#include "moving_average.h"
#include "temp.h"
#include "access.h"
#include "common.h"
#include "wireless.h"

/*Defines */
#define DEBUG 0
#define USER_BTN 0x0001 /*!<Defines the bit location of the user button*/


/*Global Variables*/
uint8_t sampleACCFlag = 0x01; /**<A flag variable for sampling, restricted to a value of 0 or 1*/
uint8_t buttonState = 0; /**<A variable that represents the current state of the button*/
uint8_t dmaFlag = 0; /**<A flag variable that represent the DMA flag*/

uint8_t tx[7] = {0x29|0x40|0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Transmission buffer for ACC for DMA*/
uint8_t rx[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Receive buffer for ACC for DMA*/

uint8_t const* txptr = &tx[0];
uint8_t* rxptr = &rx[0];

//Declare global variables externed in common.h
uint8_t txWireless[WIRELESS_BUFFER_SIZE]; /**<Transmission buffer for Wireless for DMA*/
uint8_t rxWireless[WIRELESS_BUFFER_SIZE]; /**<Receive buffer for Wireless for DMA*/

float accCorrectedValues[3];
float angles[2];
int32_t accValues[3];

//Define semaphores for global variable externed in common.h
osSemaphoreDef(accCorrectedValues)
osSemaphoreId accId;

osSemaphoreDef(txWireless)
osSemaphoreId txId;

osSemaphoreDef(rxWireless)
osSemaphoreId rxId;

/*Function Prototypes*/

/**
*@brief A function that runs the display user interface
*@retval None
*/
void displayUI(void);

/**
*@brief A function that flashes the LEDs if the pitch and roll positions are the same
*@retval None
*/
void displayPitchRoll(void);

/*!
 @brief Thread to perform the accelerometer data processing
 @param argument Unused
 */
void accelerometerThread(void const * argument);
void wirelessThread(void const * argument);

//Thread structure for above thread
osThreadDef(accelerometerThread, osPriorityNormal+1, 1, 0);
osThreadDef(wirelessThread, osPriorityNormal+1, 1, 0);

osThreadId aThread; //Accelerometer thread ID
osThreadId wThread; //Wireless thread ID


/**
*@brief The main function that creates the processing threads and displays the UI
*@retval An int
*/
int main (void) {	

	#if !DEBUG
	//Create necessary semaphores
	accId = osSemaphoreCreate(osSemaphore(accCorrectedValues), 1);
	
	initIO(); //Enable LEDs and button
	initTim3(); //Enable Tim3 at 100Hz
	initACC(); //Enable the accelerometer
	initDMA(); //Enable DMA for the accelerometer
	//initEXTIACC(); //Enable tap interrupts via exti0
	initEXTIButton(); //Enable button interrupts via exti1
	//initWireless(); //Enable the wireless module
	
	// Start threads
	
	//aThread = osThreadCreate(osThread(accelerometerThread), NULL);
	wThread = osThreadCreate(osThread(accelerometerThread), NULL);

	displayUI(); //Main display function
	#endif
}
	

void accelerometerThread(void const * argument){
  uint8_t i = 0; //Counting variables
	
	//Create structures for moving average filter
	AVERAGE_DATA_TYPEDEF dataX;
	AVERAGE_DATA_TYPEDEF dataY;
	AVERAGE_DATA_TYPEDEF dataZ;
	
	//Intialize structures for moving average filter
	movingAverageInit(&dataX);
	movingAverageInit(&dataY);
	movingAverageInit(&dataZ);
	
	GPIO_ResetBits(GPIOE, (uint16_t)0x0008); //Lower CS line
	//stream0 is rx, stream3 is tx

	//DMA2_Stream0->M0AR = (uint32_t)rxptr;
	DMA2_Stream0->NDTR = 7;
	DMA2_Stream0->M0AR = (uint32_t)rx;

	//DMA2_Stream3->M0AR = (uint32_t)txptr;
	DMA2_Stream3->NDTR = 7;
	DMA2_Stream3->M0AR = (uint32_t)tx;
	
  DMA2_Stream3->CR |= DMA_SxCR_EN;
	DMA2_Stream0->CR |= DMA_SxCR_EN;

	//Real-time processing of data
	while(1){
		
		osSignalWait(sampleACCFlag, osWaitForever ); //Wait to sample

    #if !DEBUG
		if(dmaFlag){

			osSemaphoreWait(accId, osWaitForever); //Have exclusive access to temperature
			
			int32_t* out = &accValues[0];
			//Scale the values from DMA to the actual values
			for(i=0; i<0x03; i++)
			{
					*out =(int32_t)(18 *  (int8_t)rx[2*i +1]);
					out++;
			}		

			//Filter ACC values
			calibrateACC(accValues, accCorrectedValues); //Calibrate the accelerometer	
			
			accCorrectedValues[0] = movingAverage(accCorrectedValues[0], &dataX);
			accCorrectedValues[1] = movingAverage(accCorrectedValues[1], &dataY);
			accCorrectedValues[2] = movingAverage(accCorrectedValues[2], &dataZ);
			
			osSemaphoreRelease(accId); //Release exclusive access
			
			GPIO_ResetBits(GPIOE, (uint16_t)0x0008); //Lower CS line
			//stream0 is rx, stream3 is tx

			DMA2_Stream0->M0AR = (uint32_t)rxptr;
			DMA2_Stream3->M0AR = (uint32_t)txptr;

			DMA2_Stream0->NDTR = 7;
			DMA2_Stream3->NDTR = 7;
			DMA2_Stream3->CR |= DMA_SxCR_EN;
			DMA2_Stream0->CR |= DMA_SxCR_EN;
			
			dmaFlag = 0; //Clear DMA flag0
			osSignalClear(aThread, sampleACCFlag); //Clear the sample flag
		}
		#endif
	}
}

void wirelessThread(void const * argument){
	
}
/**
*@brief A function that runs the display user interface
*@retval None
*/
void displayUI(void)
{
	uint8_t LEDState = 0; //Led state variable
	float acceleration[3]; //acceleration variable
	
	while(1){
		switch(buttonState){
			case 0: 
				//Receive data and display LEDS based on accelerometer
			break;
		
			case 1:
				//Transmit data 
			break;
		}
	}
}

void displayPitchRoll(void){
	
	//Access accCorrected values with access function
	//Access recieved values with access function
	//Compare and set LEDs accordingly
	
}

/**
*@brief An interrupt handler for EXTI0
*@retval None
*/
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET){
	buttonState = 1 - buttonState;	//Change the current tap state
	EXTI_ClearITPendingBit(EXTI_Line0);	//Clear the EXTI0 interrupt flag
    }
}

/**
*@brief An interrupt handler for Tim3
*@retval None
*/
void TIM3_IRQHandler(void)
{
	osSignalSet(aThread, sampleACCFlag);
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //Clear the TIM3 interupt bit
}

/**
*@brief An interrupt handler for DMA2_Stream0
*@retval None
*/
void DMA2_Stream0_IRQHandler(void)
{
	dmaFlag = 1;				//Set flag for accelerometer sampling
	
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0); //Clear the flag for transfer complete
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
    
  GPIO_SetBits(GPIOE, (uint16_t)0x0008);  //Raise CS Line
	
	DMA_Cmd(DMA2_Stream0, DISABLE); /// RX
  DMA_Cmd(DMA2_Stream3, DISABLE); /// TX
}
