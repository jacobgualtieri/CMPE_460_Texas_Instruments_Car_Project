#include <stdint.h>
#ifndef _CAMERA_H_
#define _CAMERA_H_
#endif

typedef struct center_of_mass {
    unsigned int x;
    uint16_t y;
} center_of_mass_t;

void INIT_Camera(void);
void readCameraData(uint16_t* raw_camera_data);
void MovingAverage(uint16_t* line_data, uint16_t* smoothed_line);
void FindCenterOfMass(uint16_t* smoothed_line, center_of_mass_t* stat_collection);
