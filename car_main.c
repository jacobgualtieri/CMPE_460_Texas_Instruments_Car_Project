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
uint16_t avg_line[128];
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

void average(uint16_t* line_data, uint16_t* avg_line_data){
    int j = 0;
    unsigned long accum_left = 0;
    unsigned long accum_right = 0;

    for (j = 0; j < 128; j++){
        if (j < 64)
            accum_left += line_data[j];
        else
            accum_right += line_data[j];
    }

    for (j = 0; j < 128; j++){
        if (j < 64)
            avg_line_data[j] = accum_left/80;
        else
            avg_line_data[j] = accum_right/80;
    }
}

int determine_direction(uint16_t* avg_line_data){
    int retVal = 0;
    retVal = (avg_line_data[0] > avg_line_data[64]) ? 1 : 0;
    return retVal;
}


int main(void){
    int direction = 0;
    int j = 0;

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
        OLED_draw_line(1, 1, (unsigned char *)"Hello World");   // casting strings to (unsigned char *) bc keil is stupid
        OLED_draw_line(2, 2, (unsigned char *)"How are you?");
        OLED_draw_line(3, 3, (unsigned char *)"Goodbye");
        OLED_write_display(OLED_TEXT_ARR);
        for(j = 0; j < 1024; j++){ OLED_TEXT_ARR[j] = 0; }
        msdelay(1000);
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
                OLED_display_clear();
                average(line, avg_line);
                direction = determine_direction(avg_line);
                // if (direction){
                //     OLED_draw_line(1, 1, (unsigned char *)"right ");
                // } else {
                //     OLED_draw_line(1, 1, (unsigned char *)"left  ");
                // }
                //OLED_write_display(OLED_TEXT_ARR);
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
