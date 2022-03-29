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

void slope_finder(uint16_t* line_data, line_stats_t* stat_collection){

    int j = 0;
    int dy = 0;
    int max_dy = 0;
    int max_idx = 0;
    int min_dy = 0;
    int min_idx = 0;

    for (j = 0; j < 127; j++){

        dy = line_data[j+1] - line_data[j];

        if (max_dy < dy){
            max_dy = dy;
            max_idx = j;
        } else {
            max_dy = max_dy;
            max_idx = max_idx;
        }

        if (dy < min_dy){
            min_dy = dy;
            min_idx = j;
        } else {
            min_dy = min_dy;
            min_idx = min_idx;
        }
    }

    // save results to Line_stats struct
    stat_collection->right_slope_index = min_idx;
    stat_collection->right_slope_amount = min_dy;
    stat_collection->left_slope_index = max_idx;
    stat_collection->left_slope_amount = max_dy;
}

/**
 * @brief Calculates the 5-point moving average of the camera line data
 * and determines the maximum value detected by the camera
 * 
 * @param line_data raw camera data
 * @param smoothed_line smoothed camera data after averaging
 * @return uint16_t the maximum value of the smoothed camera data
 */
void MovingAverageWithShift(uint16_t* line_data, uint16_t* smoothed_line, line_stats_t* stat_collection, int shift_amt){
    int i;
    uint16_t five_p_avg;
    uint16_t max = 0;
    uint16_t min = 16383;

    for(i = 2; i < 126; i++){
        five_p_avg = line_data[i+2]/5 + line_data[i+1]/5 + line_data[i]/5 + line_data[i-1]/5 + line_data[i-2]/5;
        if ((0 <= (i+shift_amt)) && ((i+shift_amt) < 128))
            smoothed_line[i+shift_amt] = five_p_avg;

        if (max < five_p_avg)
            max = five_p_avg;
        else if (five_p_avg < min)
            min = five_p_avg;
    }

    if (0 == shift_amt){
        smoothed_line[0] = smoothed_line[2];
        smoothed_line[1] = smoothed_line[2];
        smoothed_line[126] = smoothed_line[125];
        smoothed_line[127] = smoothed_line[125];
    }
    else if ((126 + (shift_amt)) < 128){
        for (i = (126 + (shift_amt)); i < 128; i++){
            smoothed_line[i] = smoothed_line[125+shift_amt];
        }
    }
    else if (0 < (2+shift_amt)){
        for (i = 0; i < (2+shift_amt); i++){
            smoothed_line[i] = smoothed_line[2+shift_amt];
        }
    }
    
    stat_collection->max = max;
    stat_collection->min = min;
}

/**
 * @brief splits the camera data in half and calculates the average of each half
 *
 * @param line_data the camera data
 * @param avg_line_data the averaged halves of camera data
 * avg_line_data is a 128 point array so it can be easily rendered on the OLED screen
 * Loop unrolling may be helpful if we ever need this to run faster
 * - Added 1 degree of loop unrolling
 */
void split_average(uint16_t* line_data, uint16_t* avg_line_data){
    int j;
    unsigned long accum_left = 0;
    unsigned long accum_right = 0;
    uint16_t left_avg = 0;
    uint16_t right_avg = 0;

    for (j = 0; j < 64; j++){
        accum_left += line_data[j];
        accum_right += line_data[j+64];
    }

    left_avg = accum_left/64;
    right_avg = accum_right/64;

    for (j = 0; j < 64; j++){
        avg_line_data[j] = left_avg;
        avg_line_data[j+64] = right_avg;
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
    int retVal = 0;
    int margin = 2000;
    
    uint16_t left_amt, right_amt;
    
    left_amt = avg_line_data[0];
    right_amt = avg_line_data[64];

    if (left_amt > (right_amt + margin)){         // turn left
        retVal = 1;
    }
    else if ((left_amt + margin) < right_amt){  // turn right
        retVal = -1;
    }
    else {          // go straight
        retVal = 0;
    }

    return retVal;
}
