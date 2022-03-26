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
#include "Camera.h"

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
 */
void myDelay(void){
	volatile int j;
	for (j = 0; j < 800000; j++){}
}

/**
 * @brief msdelay function from Lab 5
 */
void msdelay(int delay){
    int i,j;
    for(i=0;i<delay;i++)
        for(j=0;j<16000;j++);
}
/**
 * @brief initializes LED1, LED2, UART, and OLED
 */
void init(void){
    DisableInterrupts();
    LED1_Init();
    LED2_Init();
    uart0_init();
    uart2_init();
    INIT_Camera();

    #ifdef USE_OLED
        OLED_Init();
        OLED_display_on();
        OLED_display_clear();
        OLED_display_on();
    #else
        #ifdef TEST_OLED            // Cascaded ifdef to avoid initializing OLED screen twice
                OLED_Init();
                OLED_display_on();
                OLED_display_clear();
                OLED_display_on();
            #endif
    #endif
}

int main(void){
    int direction = 0;  //  degree that the wheels should turn
    int j = 0;

    /* Initializations */
    init();
    
    /* Test OLED Display*/
    #ifdef TEST_OLED
        OLED_draw_line(1, 1, (unsigned char *)"Hello World");   // casting strings to (unsigned char *) bc keil is stupid
        OLED_draw_line(2, 2, (unsigned char *)"How are you?");
        OLED_draw_line(3, 3, (unsigned char *)"Goodbye");
        OLED_write_display(OLED_TEXT_ARR);
        msdelay(1000);
        for(j = 0; j < 1024; j++){ OLED_TEXT_ARR[j] = 0; }
        OLED_display_clear();
    #endif

    /* Begin Infinite Loop */
    EnableInterrupts();

    for (;;){

        // parse camera data
        if (g_sendData == TRUE){
			LED1_On();

            // TODO: Parse camera data (line)
            split_average(line, avg_line);
            direction = determine_direction(avg_line);

            // render camera data onto the OLED display
            #ifdef USE_OLED
                DisableInterrupts();
                OLED_display_clear();
                OLED_DisplayCameraData(avg_line);
                // if (direction){
                //     OLED_draw_line(1, 1, (unsigned char *)"right ");
                // } else {
                //     OLED_draw_line(1, 1, (unsigned char *)"left  ");
                // }
                // OLED_write_display(OLED_TEXT_ARR);
                EnableInterrupts();
            #endif
            
			g_sendData = FALSE;
			LED1_Off();
		}

        // Turn the servo motor


        // Set the speed the DC motors should spin


        
		// do a small delay
		myDelay();
    }


    // TODO: Use this formatting instead for a clear, understandable main loop
    for(;;){

        // Read the line scan camera data
        readCameraData(line);


        // Smooth and filter the raw data


        // Determine the degree to turn the servo
        direction = determine_direction(avg_line);


        // Turn the servo motor


        // Set the speed the DC motors should spin




        myDelay();
    }
}
