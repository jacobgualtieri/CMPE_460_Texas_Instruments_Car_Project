#include <stdlib.h>
#include <stdint.h>

#define HISTORY_LENGTH 2

#define STRAIGHT   0.075
#define FULL_LEFT  0.05
#define FULL_RIGHT 0.1

/* Midpoint Calculation */
#define MIDPOINT(L_IDX,R_IDX) (((L_IDX) + (R_IDX))/2)

double SteeringPID(int left_edge, int right_edge);
double Integrate(double* previous_values);
