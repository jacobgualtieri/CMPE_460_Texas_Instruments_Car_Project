#include <stdlib.h>
#include <stdint.h>
#include "Steering_PID.h"
#include "leds.h"

extern double ERROR_HISTORY[HISTORY_LENGTH];

/**
 * @brief Integrates via trapezoid rule
 * 
 */
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
    int track_midpoint = 64;
    double error = 0.0;
    double proportional_gain = 0.0;
    double integral_gain = 0.0;
    double derivative_gain = 0.0;
    double new_servo_position = 0.075;

    // Find the current midpoint of the track
    track_midpoint = MIDPOINT(left_edge, right_edge);

    // Determine error
    error = (double)(64 - track_midpoint);

    // Push error term into history array
    ERROR_HISTORY[1] = ERROR_HISTORY[0];
    ERROR_HISTORY[0] = error;

    // First determine proportional gain
    proportional_gain = k_p * error;

    // Determine integral gain
    integral_gain = k_i * Integrate(ERROR_HISTORY);

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

    /* LED stuff for debugging */
    if (new_servo_position <= 0.065){
        LED2_Red();
    }
    else if ((0.065 < new_servo_position) && (new_servo_position < 0.070)){
        LED2_Magenta();
    }
    else if ((0.08 < new_servo_position) && (new_servo_position < 0.085)){
        LED2_Cyan();
    }
    else if (0.085 <= new_servo_position){
        LED2_Blue();
    }
    else {
        LED2_Off();
    }

    return new_servo_position;
}
