#include <stdint.h>
#include "Common.h"
#include "ControlPins.h"
#include "ADC14.h"
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
 * @brief Calculates the 7-point moving average of the camera line data.
 * This is a digital low pass filter with kernel size 7 and a gain of 1
 * 
 * @param line_data raw camera data
 * @param smoothed_line smoothed camera data after averaging
 */
void MovingAverage(uint16_t* line_data, uint16_t* smoothed_line){
    uint16_t avg;    // 7 point average value

    for(int i = 3; i < 125; i++){
        avg = line_data[i+3]/7 + line_data[i+2]/7 + line_data[i+1]/7 + line_data[i]/7 + line_data[i-1]/7 + line_data[i-2]/7 + line_data[i-3]/7;
        smoothed_line[i] = avg;
    }

    //  First 3 values in smoothed array get set to the first element to get averaged
    smoothed_line[0] = smoothed_line[3];
    smoothed_line[1] = smoothed_line[3];
    smoothed_line[2] = smoothed_line[3];

    //  Last 3 values in the array get set to the last element to get averaged
    smoothed_line[125] = smoothed_line[124];
    smoothed_line[126] = smoothed_line[124];
    smoothed_line[127] = smoothed_line[124];
}
