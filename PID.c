#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "PID.h"
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

double GenericPID(pid_values_t pid_params, double desired, double actual, double error_terms [HISTORY_LENGTH]){
    double error = 0.0;
    double proportional_gain = 0.0;
    double integral_gain = 0.0;
    double derivative_gain = 0.0;
    double new_setpoint = 0.0;

    /*
    e(n)   = error
    e(n-1) = error[0]
    e(n-2) = error[1]
    */
    // Determine error
    error = desired - actual;

    // First determine proportional gain
    proportional_gain = pid_params.kp * (error - error_terms[0]);

    // Determine integral gain
    integral_gain = pid_params.ki * ((error + error_terms[0])/2.0);

    // Finally determine derivative gain
    derivative_gain = pid_params.kd * (error - (2.0*error_terms[0]) + error_terms[1]);

    new_setpoint = actual + (proportional_gain + integral_gain + derivative_gain);
    
    // Push error term into history array
    error_terms[2] = error_terms[1];
    error_terms[1] = error_terms[0];
    error_terms[0] = error;

    return new_setpoint;
}

/**
 * From PID Notes
 * - Higher k_p values result in obtaining the desired steering angle quickly,
 *   but with excessive oscillations
 * - Start with k_p = 0.5
 */
double SteeringPID(pid_values_t pid_params, double desired, double actual){
    double new_servo_position = 0.075;

    new_servo_position = GenericPID(pid_params, desired, actual, STEERING_ERROR_HISTORY);

    // Prevent control loop from exceeding servo range
    if (new_servo_position < FULL_RIGHT){
        new_servo_position = FULL_RIGHT;
    }
    else if (FULL_LEFT < new_servo_position){
        new_servo_position = FULL_LEFT;
    }

    /* LED stuff for debugging */
    if (new_servo_position < 0.059){
        LED2_Red();
    }
    else if (new_servo_position < 0.070){
        LED2_Magenta();
    }
    else if ((0.08 < new_servo_position) && (new_servo_position <= 0.085)){
        LED2_Cyan();
    }
    else if (0.085 < new_servo_position){
        LED2_Blue();
    }
    else {
        LED2_Off();
    }

    return new_servo_position;
}

double DrivingPID(pid_values_t pid_params, double desired, double actual){

    double new_speed = 0.0;

    new_speed = GenericPID(pid_params, desired, actual, DRIVING_ERROR_HISTORY);

    // Prevent control loop from exceeding servo range
    if (new_speed < 10.0){
        new_speed = 10.0;
    }

    return new_speed;
}
