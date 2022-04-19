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
 * @brief finds the 'center of mass' of the car data used to determine the
 * center position of the track
 * 
 * @param smoothed_line the smoothed camera data
 * @return unsigned int 
 */
void FindCenterOfMass(uint16_t* smoothed_line, center_of_mass_t* stat_collection){
    int i = 0;
    double accum = 0.0;
    double accum_y = 0.0;
    unsigned int center = 64;

    for (i = 0; i < 128; i++){
        accum_y += smoothed_line[i];
        accum += (smoothed_line[i] * i);
    }

    center = (unsigned int)(accum / accum_y);

    stat_collection->x = center;    // divide by 128
    stat_collection->y = smoothed_line[center];
}

/**
 * @brief finds the indexes of the left and right lines of the "track"
 * from the camera data
 *
 * @param line_data smoothed camera data
 * @param stat_collection contains values and indexes of the max and min
 * values of the derivative (slope) of the input data
 */
// void slope_finder(uint16_t* line_data, line_stats_t* stat_collection){
//     int j;      // counter
//     int dy;     // delta y, or slope between two points
//     int max_dy = 0;     // max slope (largest positive slope value)
//     int max_idx = 0;    // index of max slope
//     int min_dy = 0;     // min slope (smallest negative slope value)
//     int min_idx = 0;    // index of min slope

//     for (j = 0; j < 127; j++){

//         dy = line_data[j+1] - line_data[j]; // calculate slope = ( [j+1] - [j] ) / 1

//         if (max_dy < dy){   // update max values
//             max_dy = dy;
//             max_idx = j;
//         } else {
//             max_dy = max_dy;
//             max_idx = max_idx;
//         }

//         if (dy < min_dy){   // update min values
//             min_dy = dy;
//             min_idx = j;
//         } else {
//             min_dy = min_dy;
//             min_idx = min_idx;
//         }
//     }

//     // save results to line_stats struct
//     stat_collection->right_slope_index = min_idx;
//     stat_collection->right_slope_amount = min_dy;
//     stat_collection->left_slope_index = max_idx;
//     stat_collection->left_slope_amount = max_dy;
// }

/**
 * @brief Calculates the 5-point moving average of the camera line data
 * and determines the maximum value detected by the camera
 * 
 * @param line_data raw camera data
 * @param smoothed_line smoothed camera data after averaging
 * @param stat_collection contains values and indexes of the max and min
 * values of the raw camera data
 */
void MovingAverage(uint16_t* line_data, uint16_t* smoothed_line){
    int i;
    uint16_t five_p_avg;    // 5 point average value

    for(i = 2; i < 126; i++){
        five_p_avg = line_data[i+2]/5 + line_data[i+1]/5 + line_data[i]/5 + line_data[i-1]/5 + line_data[i-2]/5;
        smoothed_line[i] = five_p_avg;
    }

    smoothed_line[0] = line_data[2];
    smoothed_line[1] = line_data[2];
    smoothed_line[126] = line_data[125];
    smoothed_line[127] = line_data[125];
}
