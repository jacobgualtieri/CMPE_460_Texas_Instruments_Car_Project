#include <stdint.h>

#ifndef _CAMERA_H_
#define _CAMERA_H_
#endif

typedef struct max_and_mins {
    uint16_t max;
    uint16_t min;
} max_and_mins_t;

void INIT_Camera(void);
void readCameraData(uint16_t* raw_camera_data);
max_and_mins_t MovingAverage(uint16_t* line_data, uint16_t* smoothed_line);
void split_average(uint16_t* line_data, uint16_t* avg_line_data);
void slope_finder(uint16_t* line_data, int* result_array);
int determine_direction(uint16_t* avg_line_data);
