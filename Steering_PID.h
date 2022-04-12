#include <stdlib.h>
#include <stdint.h>

#ifndef _STEERING_PID_
#define _STEERING_PID_
#endif

#define HISTORY_LENGTH 3

#define STRAIGHT   0.075
#define FULL_LEFT  0.11
#define FULL_RIGHT 0.05

/* Midpoint Calculation */
#define MIDPOINT(L_IDX,R_IDX) (((L_IDX) + (R_IDX))/2)

#define UPDATE_PID(PID_STRUCT,NEW_KP,NEW_KI,NEW_KD){ \
    (PID_STRUCT).kp = (NEW_KP); \
    (PID_STRUCT).ki = (NEW_KI); \
    (PID_STRUCT).kd = (NEW_KD); \
}

typedef struct pid_values {
    float kp;
    float ki;
    float kd;
} pid_values_t;


void PrintSteeringValues(pid_values_t steering_values);
void PrintDrivingValues(pid_values_t driving_values);
double Integrate(double* previous_values);
double GenericPID(pid_values_t pid_params, double target, double setpoint, double error_terms [HISTORY_LENGTH]);
double SteeringPID(pid_values_t pid_params, double desired, double actual);
double DrivingPID(pid_values_t pid_params, double desired, double actual);
