#include <stdint.h>

#ifndef _CAMERA_H_
#define _CAMERA_H_
#endif

void INIT_Camera(void);
void readCameraData(uint16_t* raw_camera_data);
void split_average(uint16_t* line_data, uint16_t* avg_line_data);
int determine_direction(uint16_t* avg_line_data);
