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
 * @brief Initializes camera signals
 */
void INIT_Camera(void){
    g_sendData = FALSE;
    ControlPin_SI_Init();
    ControlPin_CLK_Init();
    ADC0_InitSWTriggerCh6();
}

void readCameraData(uint16_t* raw_camera_data){
    // TODO: Re-evaluate the purpose of this function
    //  I don't think we even use this anymore
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
 * @brief finds the indexes of the left and right lines of the "track"
 * from the camera data
 *
 * @param line_data smoothed camera data
 * @param stat_collection contains values and indexes of the max and min
 * values of the derivative (slope) of the input data
 */
void slope_finder(uint16_t* line_data, line_stats_t* stat_collection){
    int j;      // counter
    int dy;     // delta y, or slope between two points
    int max_dy = 0;     // max slope (largest positive slope value)
    int max_idx = 0;    // index of max slope
    int min_dy = 0;     // min slope (smallest negative slope value)
    int min_idx = 0;    // index of min slope

    for (j = 0; j < 127; j++){

        dy = line_data[j+1] - line_data[j]; // calculate slope = ( [j+1] - [j] ) / 1

        if (max_dy < dy){   // update max values
            max_dy = dy;
            max_idx = j;
        } else {
            max_dy = max_dy;
            max_idx = max_idx;
        }

        if (dy < min_dy){   // update min values
            min_dy = dy;
            min_idx = j;
        } else {
            min_dy = min_dy;
            min_idx = min_idx;
        }
    }

    // save results to line_stats struct
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
 * @param stat_collection contains values and indexes of the max and min
 * values of the raw camera data
 */
void MovingAverage(uint16_t* line_data, uint16_t* smoothed_line, line_stats_t* stat_collection){
    int i;
    uint16_t five_p_avg;    // 5 point average value
    uint16_t max = 0;
    uint16_t min = 16383;

    for(i = 2; i < 126; i++){
        five_p_avg = line_data[i+2]/5 + line_data[i+1]/5 + line_data[i]/5 + line_data[i-1]/5 + line_data[i-2]/5;
        smoothed_line[i] = five_p_avg;

        if (max < smoothed_line[i])
            max = smoothed_line[i];
        else if (smoothed_line[i] < min)
            min = smoothed_line[i];
    }

    smoothed_line[0] = line_data[2];
    smoothed_line[1] = line_data[2];
    smoothed_line[126] = line_data[125];
    smoothed_line[127] = line_data[125];

    // TODO: Why do we do this here? Why do we not keep track of the indexes too?
    stat_collection->max = max;
    stat_collection->min = min;
}

/**
 * @brief Splits the camera data in half and calculates the average of each half
 *
 * @param line_data raw camera data
 * @param avg_line_data the averaged halves of camera data. It is a 128 point array
 * so it can be easily rendered on the OLED screen. Loop unrolling may be helpful
 * if we ever need this to run faster
 *
 * - Added 1 degree of loop unrolling
 */
void split_average(uint16_t* line_data, uint16_t* avg_line_data){
    int j;
    unsigned long accum_left = 0;
    unsigned long accum_right = 0;
    uint16_t left_avg;
    uint16_t right_avg;

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
 * @brief Determines which half of the camera data has a greater average
 *
 * @param avg_line_data the averaged halves of camera data
 * @return int retVal
 *
 * If retVal = -1, turn to the left
 * If retVal = 0, go straight (don't change direction)
 * If retVal = 1, turn to the right
 */
int determine_direction(uint16_t* avg_line_data){
    int retVal;
    int margin = 2000;

    uint16_t left_amt, right_amt;

    left_amt = avg_line_data[0];    // left average
    right_amt = avg_line_data[64];  // right average

    if (left_amt > (right_amt + margin)){   // turn left
        retVal = 1;
    }
    else if ((left_amt + margin) < right_amt){  // turn right
        retVal = -1;
    }
    else {  // go straight
        retVal = 0;
    }

    return retVal;
}
