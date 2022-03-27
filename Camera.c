#include <stdint.h>
#include <stdio.h>
#include "Common.h"
#include "ControlPins.h"
#include "ADC14.h"
#include "leds.h"
#include "uart.h"
#include "Camera.h"

extern BOOLEAN g_sendData;

/**
 * @brief camera initialization function
 */
void INIT_Camera(void){
    g_sendData = FALSE;
    ControlPin_SI_Init();
    ControlPin_CLK_Init();
    ADC0_InitSWTriggerCh6();
}

void readCameraData(uint16_t* raw_camera_data){
    char str[100];
    int i;
    if (g_sendData == TRUE){
        LED1_On();

        // send the array over uart
        sprintf(str,"%i\n\r",-1); // start value
        uart0_put(str);
        uart2_put(str);
        for (i = 0; i < 128; i++){
            sprintf(str,"%i\n\r", raw_camera_data[i]);
            uart0_put(str);
            uart2_put(str);
        }
        sprintf(str,"%i\n\r",-2); // end value
        uart0_put(str);
        uart2_put(str);
        LED1_Off();
        g_sendData = FALSE;
    }
}

/**
 * @brief splits the camera data in half and calculates the average of each half
 *
 * @param line_data the camera data
 * @param avg_line_data the averaged halves of camera data
 * avg_line_data is a 128 point array so it can be easily rendered on the OLED screen
 * Loop unrolling may be helpful if we ever need this to run faster
 */
void split_average(uint16_t* line_data, uint16_t* avg_line_data){
    int j;
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
            avg_line_data[j] = accum_left/64;
        else
            avg_line_data[j] = accum_right/64;
    }
}

/**
 * @brief determines which half of the camera data has a greater average
 *
 * @param avg_line_data the averaged halves of camera data
 * @return int
 * If retVal = -1, turn to the left
 * If retVal = 0, go straight (don't change direction)
 * If retVal = 1, turn to the right
 */
int determine_direction(uint16_t* avg_line_data){
    int degree = 0;
    int margin = 1000;

    if (avg_line_data[0] > (avg_line_data[64] + 1000)){         // turn left
        degree = 1;
    }
    else if ((avg_line_data[0] + margin) < avg_line_data[64]){  // turn right
        degree = -1;
    }
    else {          // go straight
        degree = 0;
    }

    return degree;
}
