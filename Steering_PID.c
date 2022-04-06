#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "Steering_PID.h"
#include "leds.h"
#include "uart.h"

extern double STEERING_ERROR_HISTORY[HISTORY_LENGTH];
extern double DRIVING_ERROR_HISTORY[HISTORY_LENGTH];

void PrintSteeringValues(pid_values_t pid){
    char string [100];
    sprintf(string, "Steering values: Kp = %.3f, Ki = %.3f, Kd = %.3f\n\r", pid.kp, pid.ki, pid.kd);
    uart2_put(string);
    uart0_put(string);
}

void PrintDrivingValues(pid_values_t pid){
    char string [100];
    sprintf(string, "Driving values: Kp = %.3f, Ki = %.3f, Kd = %.3f\n\r", pid.kp, pid.ki, pid.kd);
    uart2_put(string);
    uart0_put(string);
}

/**
 * @brief Integrates via trapezoid rule
 * 
 */
double Integrate(double previous_values [HISTORY_LENGTH]){
    int i = 0;
    double accum = 0.0;

    for(i = 0; i < HISTORY_LENGTH; i++)
        accum += previous_values[i];

    accum = accum / (double)HISTORY_LENGTH;

    return accum;
}

double GenericPID(pid_values_t pid_params, double target, double setpoint, double error_terms [HISTORY_LENGTH]){
    double error = 0.0;
    double proportional_gain = 0.0;
    double integral_gain = 0.0;
    double derivative_gain = 0.0;
    double new_setpoint = 0.0;

    // Determine error
    error = target - setpoint;

    // Push error term into history array
    error_terms[1] = error_terms[0];
    error_terms[0] = error;

    // First determine proportional gain
    proportional_gain = pid_params.kp * error;

    // Determine integral gain
    integral_gain = pid_params.ki * Integrate(error_terms);

    // Finally determine derivative gain
    derivative_gain = 0.0;

    new_setpoint = setpoint + (proportional_gain + integral_gain + derivative_gain);

    return new_setpoint;
}

/**
 * From PID Notes
 * - Higher k_p values result in obtaining the desired steering angle quickly,
 *   but with excessive oscillations
 * - Start with k_p = 0.5
 */
double SteeringPID(int left_edge, int right_edge, pid_values_t pid_params){
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

    new_servo_position = GenericPID(pid_params, 64.0, (double)track_midpoint, STEERING_ERROR_HISTORY);

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
