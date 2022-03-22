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

#define USE_OLED
#define TEST_OLED

// line stores the current array of camera data
extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
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
 * @brief msdelay function from Lab 5
 *
 */
void msdelay(int delay){
    int i,j;
    for(i=0;i<delay;i++)
        for(j=0;j<16000;j++);
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

double average(BOOLEAN side, uint16_t* line_data){
    int j = 0;
    int offset = 0;
    unsigned long accum = 0;
    double avg = 0.0;

    offset = (side == 0) ? 0 : 64;

    for (j = offset; j < (64+offset); j++){
        accum += line_data[j];
    }

    avg = (double)accum / 64.0;
    return avg;
}


int main(void){

    /* Initializations */
    DisableInterrupts();

    #ifdef USE_OLED
        OLED_Init();
	    OLED_display_on();
	    OLED_display_clear();
	    OLED_display_on();
    #else
        #ifdef TEST_OLED
            OLED_Init();
            OLED_display_on();
            OLED_display_clear();
            OLED_display_on();
        #endif
    #endif
    LED1_Init();
    LED2_Init();
    INIT_Camera();
    
    /* Test OLED Display*/
    #ifdef TEST_OLED
        OLED_draw_line(1, 1, "Hello World");
        OLED_draw_line(2, 2, "How are you?");
        OLED_draw_line(3, 3, "Goodbye");
        OLED_write_display(OLED_TEXT_ARR);
        msdelay(100);
        OLED_display_clear();
    #endif

    /* Begin Infinite Loop */
    EnableInterrupts();

    for (;;){
        if (g_sendData == TRUE){
			LED1_On();

            #ifdef USE_OLED
                // render camera data onto the OLED display
                DisableInterrupts();
                OLED_DisplayCameraData(line);
                EnableInterrupts();
            #endif

            // TODO: Parse camera data (line)
                
			g_sendData = FALSE;
			LED1_Off();
		}
        
		// do a small delay
		myDelay();
    }
}
