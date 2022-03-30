#include <stdint.h>
#ifndef _CAMERA_H_
#define _CAMERA_H_
#endif

typedef struct line_stats {
    uint16_t max;
    uint16_t min;
    int right_slope_index;
    int left_slope_index;
    int right_slope_amount;
    int left_slope_amount;
} line_stats_t;

void INIT_Camera(void);
void readCameraData(uint16_t* raw_camera_data);
void MovingAverageWithShift(uint16_t* line_data, uint16_t* smoothed_line, line_stats_t* stat_collection, int shift_amt);
void slope_finder(uint16_t* line_data, line_stats_t* stat_collection);
int determine_direction(uint16_t* avg_line_data);
void split_average(uint16_t* line_data, uint16_t* avg_line_data);
