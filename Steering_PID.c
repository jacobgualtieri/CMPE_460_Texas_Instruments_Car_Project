#include <stdlib.h>
#include <stdint.h>
#include "Steering_PID.h"

extern double MIDPOINT_HISTORY[HISTORY_LENGTH];

double Integrate(double* previous_values){
    int i = 0;
    double accum = 0.0;

    for(i = 0; i < HISTORY_LENGTH; i++)
        accum += previous_values[i];

    accum = accum / (double)HISTORY_LENGTH;

    return accum;
}

/**
 * From PID Notes
 * - Higher k_p values result in obtaining the desired steering angle quickly,
 *   but with excessive oscillations
 * - Start with k_p = 0.5
 */
double SteeringPID(int left_edge, int right_edge, double k_p, double k_i){
    int track_midpoint;

    double proportional_gain = 0.0;
    double integral_gain = 0.0;
    double derivative_gain = 0.0;
    double new_servo_position = 7.5;

    track_midpoint = MIDPOINT(left_edge, right_edge);

    // First determine proportional gain
    proportional_gain = k_p * (double)(64 - track_midpoint);

    // Determine integral gain
    integral_gain = k_i * Integrate(MIDPOINT_HISTORY);

    // Finally determine derivative gain
    derivative_gain = 0.0;

    new_servo_position = proportional_gain + integral_gain + derivative_gain;

    // Prevent control loop from exceeding servo range
    if (new_servo_position < FULL_LEFT){
        new_servo_position = FULL_LEFT;
    }
    else if (FULL_RIGHT < new_servo_position){
        new_servo_position = FULL_RIGHT;
    }

    return new_servo_position;
}
