#ifndef _STEERING_PID_
#define _STEERING_PID_
#endif

#define HISTORY_LENGTH 3
#define FULL_LEFT  0.11
#define FULL_RIGHT 0.05

/* Midpoint Calculation */
#define MIDPOINT(L_IDX,R_IDX) (((L_IDX) + (R_IDX))/2)


typedef struct pid_values {
    float kp;
    float ki;
    float kd;
} pid_values_t;

double GenericPID(pid_values_t pid_params, double target, double setpoint, double error_terms [HISTORY_LENGTH]);
double SteeringPID(pid_values_t pid_params, double desired, double actual);
double DrivingPID(pid_values_t pid_params, double desired, double actual);
