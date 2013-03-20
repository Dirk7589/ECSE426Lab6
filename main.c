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
#include "fir.h"

/*Defines */
#define DEBUG 0
#define LAB4 1
#define MANUAL_FILTER 1
#define MAX_COUNTER_VALUE 5; //Maximum value for the temperature sensor to sample at 20Hz
#define USER_BTN 0x0001 /*!<Defines the bit location of the user button*/

/*Global Variables*/
static uint8_t tapState = 1; /**<A varaible that represents the current tap state*/
uint8_t sampleACCFlag = 0x01; /**<A flag variable for sampling, restricted to a value of 0 or 1*/
uint8_t sampleTempCounter = 0; /**<A counter variable for sampling the temperature sensor */
int32_t sampleTempFlag = 0x0001;
uint8_t buttonState = 0; /**<A variable that represents the current state of the button*/
uint8_t dmaFlag = 0; /**<A flag variable that represent the DMA flag*/

/*Global 2*/

uint8_t tx[7] = {0x29|0x40|0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Transmission buffer for DMA*/
uint8_t rx[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Receive buffer for DMA*/

uint8_t const* txptr = &tx[0];
uint8_t* rxptr = &rx[0];

/*Global 3*/
//Coefficients for the FIR filter
float32_t coeffPFloat[] = {
0.07416114002096 , 0.03270293265285 , 0.03863816947328 ,
0.04444276039114 , 0.0497985768307 , 0.05449897183339 ,
0.05841301164775 , 0.06134989872289 , 0.06313835392955 ,
0.06373119762273 , 0.06313835392955 , 0.06134989872289 ,
0.05841301164775 , 0.05449897183339 , 0.0497985768307 ,
0.04444276039114 , 0.03863816947328 , 0.03270293265285 ,
0.07416114002096
};

//Data Variables decleration
float temperature; 
float accCorrectedValues[3];
float angles[2];
int32_t accValues[3];

//Define semaphores
osSemaphoreDef(temperature)
osSemaphoreDef(accCorrectedValues)
osSemaphoreId tempId;
osSemaphoreId accId;

/*Function Prototypes*/

/**
*@brief A function that runs the display user interface
*@retval None
*/
void displayUI(void);

/*!
 @brief Thread to perform the temperature data processing
 @param argument Unused
 */
void temperatureThread(void const * argument);

/*!
 @brief Thread to perform the accelerometer data processing
 @param argument Unused
 */
void accelerometerThread(void const * argument);

//Thread structure for above thread
osThreadDef(temperatureThread, osPriorityNormal+1, 1, 0);
osThreadDef(accelerometerThread, osPriorityNormal+1, 1, 0);
osThreadId tThread; //Tempearture thread ID
osThreadId aThread; //Accelerometer thread ID


/**
*@brief The main function that creates the processing threads and displays the UI
*@retval An int
*/
int main (void) {	

	#if !DEBUG
	//Create necessary semaphores
	tempId = osSemaphoreCreate(osSemaphore(temperature), 1);
	accId = osSemaphoreCreate(osSemaphore(accCorrectedValues), 1);
	
	initIO(); //Enable LEDs and button
	initTempADC(); //Enable ADC for temp sensor
	initTim3(); //Enable Tim3 at 100Hz
	initACC(); //Enable the accelerometer
	initDMAACC(); //Enable DMA for the accelerometer
	initEXTIACC(); //Enable tap interrupts via exti0
	initEXTIButton(); //Enable button interrupts via exti1
	
	// Start threads
	tThread = osThreadCreate(osThread(temperatureThread), NULL);
	aThread = osThreadCreate(osThread(accelerometerThread), NULL);

	displayUI(); //Main display function
	#endif
}

void temperatureThread(void const *argument){
	
	uint16_t adcValue = 0;
	
	AVERAGE_DATA_TYPEDEF temperatureData; //Create temperature data structure
	
	movingAverageInit(&temperatureData); //Prepare filter
	
	while(1){		
		//check sample flag. This ensures the proper 20Hz sampling rate
		osSignalWait(sampleTempFlag, osWaitForever);
			
		ADC_SoftwareStartConv(ADC1); //Start conversion
		
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET); //wait for end of conversion
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC); //clear the ADC flag
	
		adcValue = ADC1->DR; //Retrieve ADC value in bits
		
		osSemaphoreWait(tempId, osWaitForever); //Have exclusive access to temperature
		
		temperature = toDegreeC(adcValue); //Determine the temperature in Celcius
		temperature = movingAverage(temperature, &temperatureData); //Filter the temperature
		
		osSemaphoreRelease(tempId); //Release access to temperature
		
		osSignalClear(tThread, sampleTempFlag);
		
	}
}
	

void accelerometerThread(void const * argument){
  uint8_t i = 0; //Counting variables
	uint8_t j = 0;
	
	//Create structures for moving average filter
	AVERAGE_DATA_TYPEDEF dataX;
	AVERAGE_DATA_TYPEDEF dataY;
	AVERAGE_DATA_TYPEDEF dataZ;
	
	//Intialize structures for moving average filter
	movingAverageInit(&dataX);
	movingAverageInit(&dataY);
	movingAverageInit(&dataZ);
	
	//Create structures for FIR filter
	#if MANUAL_FILTER
	FIR_DATA dataXFIR;
	FIR_DATA dataYFIR;
	FIR_DATA dataZFIR;
	
	firInit(&dataXFIR);
	firInit(&dataYFIR);
	firInit(&dataZFIR);
	#endif
	
	//Create structures for automatic FIR filters
	#if !MANUAL_FILTER
	int16_t pCoeffs[19]; //Create coefficients array
	uint8_t decimantionFactor = 10; //Set decimation to 10
	uint16_t numTaps = 19; //The number of coefficients
	q15_t pState[numTaps+10-1]; //Size of state variable
	
	arm_float_to_q15(&coeffPFloat[0], &pCoeffs[0], 19); //Convert coefficients from float to q15
	
	//Create instances for filters
	arm_fir_decimate_instance_q15 q15InstanceX; 
	arm_fir_decimate_instance_q15 q15InstanceY;
	arm_fir_decimate_instance_q15 q15InstanceZ;
	
	//Initialize instaces for filters
	arm_fir_decimate_init_q15(&q15InstanceX, numTaps, decimantionFactor, &pCoeffs[0], &pState[0], 10);
	arm_fir_decimate_init_q15(&q15InstanceY, numTaps, decimantionFactor, &pCoeffs[0], &pState[0], 10);
	arm_fir_decimate_init_q15(&q15InstanceZ, numTaps, decimantionFactor, &pCoeffs[0], &pState[0], 10);

	q15_t accXFixed[10], accYFixed[10], accZFixed[10]; // Buffers
	#endif
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
        #if DEBUG
        LIS302DL_ReadACC(accValues);
        osSemaphoreWait(accId, osWaitForever); //Have exclusive access to temperature
        calibrateACC(accValues, accCorrectedValues); //Calibrate the accelerometer	
            
        accCorrectedValues[0] = movingAverage(accCorrectedValues[0], &dataX);
        accCorrectedValues[1] = movingAverage(accCorrectedValues[1], &dataY);
        accCorrectedValues[2] = movingAverage(accCorrectedValues[2], &dataZ);
        
        osSemaphoreRelease(accId); //Release exclusive access
        osSignalClear(aThread, sampleACCFlag); //Clear the sample flag
        #endif
        
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
						
						#if !LAB4
            #if MANUAL_FILTER
						if(fir(accCorrectedValues[0], &dataXFIR)){
							arm_q15_to_float(&dataXFIR.result, &accCorrectedValues[0], 1); //Convert value back
						}
						if(fir(accCorrectedValues[1], &dataYFIR)){
							arm_q15_to_float(&dataYFIR.result, &accCorrectedValues[1], 1); //Convert value back
						}
						if(fir(accCorrectedValues[2], &dataZFIR)){
							arm_q15_to_float(&dataZFIR.result, &accCorrectedValues[2], 1); //Convert value back
						}
						#endif
						
						#if !MANUAL_FILTER
						
						printf("%f\n", accCorrectedValues[0]);
						
						accCorrectedValues[0] = accCorrectedValues[0] / 10000; //Scale down value
						arm_float_to_q15(&accCorrectedValues[0], &accXFixed[j], 1); //Convert X values
						accCorrectedValues[1] = accCorrectedValues[1] / 10000;
						arm_float_to_q15(&accCorrectedValues[1], &accYFixed[j], 1); //Convert Y values
						accCorrectedValues[2] = accCorrectedValues[2] / 10000;
						arm_float_to_q15(&accCorrectedValues[2], &accZFixed[j], 1); //Convert Z values
						j++; //Increase buffer to next location
						
						if(j == 9){
							arm_fir_decimate_q15(&q15InstanceX, &accXFixed[0], &accXFixed[0], 10); //Decimate
							arm_fir_decimate_q15(&q15InstanceY, &accYFixed[0], &accYFixed[0], 10);
							arm_fir_decimate_q15(&q15InstanceZ, &accZFixed[0], &accZFixed[0], 10);

							arm_q15_to_float(&accXFixed[0], &accCorrectedValues[0], 1); //Convert
							arm_q15_to_float(&accYFixed[0], &accCorrectedValues[1], 1);
							arm_q15_to_float(&accZFixed[0], &accCorrectedValues[2], 1);
							
							j = 0;
						}
						accCorrectedValues[0] = accCorrectedValues[0] * 10000;
						accCorrectedValues[1] = accCorrectedValues[1] * 10000;
						accCorrectedValues[2] = accCorrectedValues[2] * 10000;
						printf("%f\n", accCorrectedValues[0]);
						#endif
						
						#endif
						
            accCorrectedValues[0] = movingAverage(accCorrectedValues[0], &dataX);
            accCorrectedValues[1] = movingAverage(accCorrectedValues[1], &dataY);
            accCorrectedValues[2] = movingAverage(accCorrectedValues[2], &dataZ);
            
						
						
            osSemaphoreRelease(accId); //Release exclusive access
            
            GPIO_ResetBits(GPIOE, (uint16_t)0x0008); //Lower CS line
            //stream0 is rx, stream3 is tx

            DMA2_Stream0->M0AR = (uint32_t)rxptr;
            //DMA2_Stream0->M0AR = (uint32_t)rx;

            DMA2_Stream3->M0AR = (uint32_t)txptr;
           // DMA2_Stream3->M0AR = (uint32_t)txp;
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

/**
*@brief A function that runs the display user interface
*@retval None
*/
void displayUI(void)
{
	uint8_t LEDState = 0; //Led state variable
	float temp; //Temparture variable
	float acceleration[3]; //acceleration variable
	float previousValues[2] = {0,0};
	float accelerationTotals[2] = {0,0};
	
	while(1){
		switch(tapState){
			case 0: 
				switch(buttonState){
				
					case 0:
						LEDState = LEDToggle(LEDState); //Toggle LEDS
						osDelay(500);
					break;
					
					case 1:
						temp = getTemperature(); //Get the temperature from global variable
						displayTemperature(temp); //Display the temperature
					break;
				}
			break;
		
		case 1:
			
			getACCValues(acceleration); //Get the acceleration from global variable
		
			switch(buttonState){
				
					case 0:
						displayDominantAngle(acceleration); //Display the dominant angle
					break;
					
					case 1:
						displayBoardMovement(acceleration, previousValues, accelerationTotals); //display the boards movement
					break;
				}
			break;
		default:
			break;
		}

	}
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
*@brief An interrupt handler for EXTI1
*@retval None
*/
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
	tapState = 1 - tapState;	//Change the current tap state
	EXTI_ClearITPendingBit(EXTI_Line1);	//Clear the EXTI0 interrupt flag
    }
}

/**
*@brief An interrupt handler for Tim3
*@retval None
*/
void TIM3_IRQHandler(void)
{
	osSignalSet(aThread, sampleACCFlag);
	
	if(sampleTempCounter == 5){
		osSignalSet(tThread, sampleTempFlag);
		sampleTempCounter = 0;
	}
	else{
		sampleTempCounter++;												//Set flag for temperature sampling
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //Clear the TIM3 interupt bit
	//TIM_ClearFlag(TIM3, TIM_FLAG_Trigger); 				//Clear the TIM3 interupt bit
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
    //DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    GPIO_SetBits(GPIOE, (uint16_t)0x0008);  //Raise CS Line
    //DMA2_Stream0->CR |= 0; //Disable DMA
		//DMA2_Stream3->CR |= 0;
		DMA_Cmd(DMA2_Stream0, DISABLE); /// RX
    DMA_Cmd(DMA2_Stream3, DISABLE); /// TX
}
