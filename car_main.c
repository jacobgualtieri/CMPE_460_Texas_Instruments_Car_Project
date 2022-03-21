/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* Jacob Gualtieri & Zebulon Hollinger
* 3/21/2022
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "Common.h"
#include "oled.h"
#include "msp.h"
#include "uart.h"
#include "leds.h"
#include "Timer32.h"
#include "CortexM.h"
#include "ADC14.h"
#include "ControlPins.h"

// line stores the current array of camera data
uint16_t line[128];
BOOLEAN g_sendData;

/**
 * @brief simple delay function
 * 
 */
void myDelay(void){
	volatile int j;
	for (j = 0; j < 800000; j++){}
}

/**
 * @brief camera initialization function
 * 
 */
void INIT_Camera(void){
	g_sendData = FALSE;
	ControlPin_SI_Init();
	ControlPin_CLK_Init();
	ADC0_InitSWTriggerCh6();
}


int main(void){

    unsigned int i = 0;

    /* Initializations */
    DisableInterrupts();

    LED1_Init();
    LED2_Init();
    INIT_Camera();

    /* Begin Infinite Loop */
    EnableInterrupts();

    for (;;){
        if (g_sendData == TRUE){
			LED1_On();
			for (i = 0; i < 128; i++){

                // TODO: Parse camera data (line)
				//sprintf(str,"%i\n\r", line[i]);
                
			}
			g_sendData = FALSE;
			LED1_Off();
		}
		// do a small delay
		myDelay();
    }
}
