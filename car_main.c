/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460 Interfacing Digital Electronics
* Jacob Gualtieri & Zebulon Hollinger
* 3/21/2022
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "Common.h"
#include "oled.h"
#include "msp.h"
#include "uart.h"
#include "leds.h"
#include "Timer32.h"
#include "CortexM.h"
#include "ADC14.h"
#include "ControlPins.h"
#include "Camera.h"
#include "TimerA.h"
#include "PID.h"
#include "switches.h"

/* Testing and debugging */
//#define USE_OLED
//#define USE_UART
//#define TEST_OLED

/* UART */
#define UART2_RX_BUFFER_LENGTH 20

/* Servo Positions */
#define CENTER_POSITION 0.075   //  Center position of servo
#define SHARP_RIGHT     0.05    //  .005 from slight left
#define SHARP_LEFT      0.11    //  .005 from slight right

/* Directional Thresholds */
#define RIGHT_IDX_OFFSET 2  //  Shift used to account for camera mounting
#define MIDPOINT_OFFSET 0

/* Speed Settings */
#define STRAIGHTS_SPEED     30.0    //  desired speed in the straight
#define CORNERING_SPEED     27.0    //  desired speed in the corner
#define INNER_WHEEL_SLOWDOWN 4.0    //  decrease factor for inner wheel on turns

/* DC Motor Settings */
// 3 and 4 motor goes' fwd
// FORWARD: LEFT <= 3, RIGHT <= 4
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

/* Track Loss Limit */
#define TRACK_LOSS_LIMIT 3  // Stop limit if off track

// line stores the current array of camera data
extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];

/* Servo Position History Array */
double STEERING_ERROR_HISTORY[HISTORY_LENGTH] = {0.0, 0.0, 0.0};
double DRIVING_ERROR_HISTORY[HISTORY_LENGTH] = {0.0, 0.0, 0.0};

uint16_t line[128];             // raw camera data
uint16_t smoothed_line[128];    // 5-point average of raw data
BOOLEAN g_sendData;             // TRUE if camera data is ready to read
BOOLEAN running = FALSE;        // Driving control variable

#ifdef USE_UART
    char uart_buffer [20];
    char uart_rx_buffer [UART2_RX_BUFFER_LENGTH];
#endif

/**
 * @brief Function from Lab 5, delays by a specified amount
 * of time in milliseconds
 *
 * @param delay time in milliseconds to delay
 */
void msdelay(int delay){
    int i,j;
    for(i=0;i<delay;i++)
        for(j=0;j<16000;j++);
}

/**
 * @brief Initialize servo motor for steering
 */
void initSteering(void){
    // Initialize history array for PID steering to zero
    int i;
    for (i = 0; i < HISTORY_LENGTH; i++){ STEERING_ERROR_HISTORY[i] = 0.0; }

    // Initialize steering position to be straight
    // PWM -> f = 1kHz, T = 20ms, center = 1.5ms
    TIMER_A2_PWM_Init(CalcPeriodFromFrequency(1000.0), CENTER_POSITION, 1);
}

/**
 * @brief Adjusts steering based on camera input
 *
 * @param line_stats contains values and indexes of the max and min
 * values of the derivative (slope) of the input data
 * @return new servo position
 */
double adjustSteering(line_stats_t line_stats, pid_values_t pid_params){
    double servo_position;
    uint16_t left_amt, right_amt;
    int left_line_index, right_line_index;
    int track_midpoint_idx;
    int delta;

    right_line_index = line_stats.right_slope_index + RIGHT_IDX_OFFSET;
    left_line_index = line_stats.left_slope_index;
    right_amt = -1 * line_stats.right_slope_amount;
    left_amt = line_stats.left_slope_amount;
    
    if (right_line_index < left_line_index){    // Error case
        if (left_amt < right_amt){
            // Turn Left
            track_midpoint_idx = MIDPOINT(0, right_line_index);
            delta = 64-track_midpoint_idx;
        }
        else {
            // Turn Right
            track_midpoint_idx = MIDPOINT(left_line_index, 127);
            delta = track_midpoint_idx - 64;
        }
    }
    else {  // Normal cases
        track_midpoint_idx = MIDPOINT(left_line_index, right_line_index) - MIDPOINT_OFFSET;
        delta = 64-track_midpoint_idx;
    }
    
    
    if (delta == 0)
        servo_position = CENTER_POSITION;
    else
        servo_position = CENTER_POSITION + (((double)delta / 64.0) * CENTER_POSITION);
    
    
    // Prevent control loop from exceeding servo range
    if (servo_position < SHARP_RIGHT)
        servo_position = SHARP_RIGHT;
    else if (SHARP_LEFT < servo_position)
        servo_position = SHARP_LEFT;

    servo_position = SteeringPID(pid_params, 0.075, servo_position);
    TIMER_A2_PWM_DutyCycle(servo_position, 1);  // set new servo position
    
    return servo_position;
}

/**
 * @brief Initializes DC motors and enable pins on motor driver board
 */
void initDriving(void){
    uint16_t period;
    double freq = 10000.0;  //  Frequency = 10 kHz
    int i;

    // Initialize history array for PID driving to zero
    for (i = 0; i < HISTORY_LENGTH; i++){ DRIVING_ERROR_HISTORY[i] = 0.0; }

    //  EN_A and EN_B are tied to H-Bridge enable pins on the motor shield
    //  M1EN -> P3.6    M2EN -> P3.7

    // Initialize pins to GPIO Mode
    P3->SEL0 &= ~BIT6;  //  SEL0 <- '0'
    P3->SEL1 &= ~BIT6;  //  SEL1 <- '0'

    P3->SEL0 &= ~BIT7;  //  SEL0 <- '0'
    P3->SEL1 &= ~BIT7;  //  SEL1 <- '0'

    // Configure the GPIO Pins for Output
    P3->DIR |= BIT6;
    P3->DIR |= BIT7;

    // Set output to '1'
    P3->OUT |= BIT6;
    P3->OUT |= BIT7;

    period = (uint16_t) CalcPeriodFromFrequency(freq);  // calculate period
    TIMER_A0_PWM_Init(period, 0.0, 1);	// M1A -> P2.4 -> TA0.1
    TIMER_A0_PWM_Init(period, 0.0, 2);	// M1B -> P2.5 -> TA0.2
    TIMER_A0_PWM_Init(period, 0.0, 3);	// M2A -> P2.6 -> TA0.3
    TIMER_A0_PWM_Init(period, 0.0, 4);	// M2B -> P2.7 -> TA0.4
}

/**
 * @brief Adjusts speed of DC Motors based on turn angle
 */
double adjustDriving(line_stats_t line_stats, pid_values_t pid_params, double current_speed){
    uint16_t left_amt, right_amt;
    int left_line_index, right_line_index;
    int track_midpoint_idx;
    int delta;
    double new_speed = 0.0;

    if (running){
        right_line_index = line_stats.right_slope_index;
        left_line_index = line_stats.left_slope_index;
        right_amt = -1 * line_stats.right_slope_amount;
        left_amt = line_stats.left_slope_amount;

        if (right_line_index < left_line_index){    // error case
            if (left_amt < right_amt){
                // Turn left
                track_midpoint_idx = MIDPOINT(0, right_line_index);
                delta = 64-track_midpoint_idx;
            }
            else {
                // Turn Right
                track_midpoint_idx = MIDPOINT(left_line_index, 127);
                delta = track_midpoint_idx - 64;
            }
        }                                           // normal cases
        else {
            track_midpoint_idx = MIDPOINT(left_line_index, right_line_index);

            if (track_midpoint_idx < 64)
                delta = 64-track_midpoint_idx;
            else
                delta = track_midpoint_idx - 64;
        }

        if (delta < 7){
            new_speed = DrivingPID(pid_params, STRAIGHTS_SPEED, current_speed);

            TIMER_A0_PWM_DutyCycle(new_speed/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(new_speed/100.0, RIGHT_MOTOR);
        }
        else {
            new_speed = DrivingPID(pid_params, CORNERING_SPEED, current_speed);

            if (track_midpoint_idx < 64){                                           // Making a left turn
                TIMER_A0_PWM_DutyCycle((new_speed - (INNER_WHEEL_SLOWDOWN + 2.0))/100.0, LEFT_MOTOR);                            // Inner wheel
                TIMER_A0_PWM_DutyCycle((new_speed+1.0)/100.0, RIGHT_MOTOR);   // Outer wheel
            }
            else {                                                                  // Making a right turn
                TIMER_A0_PWM_DutyCycle((new_speed)/100.0, LEFT_MOTOR);    // Outer wheel
                TIMER_A0_PWM_DutyCycle((new_speed - INNER_WHEEL_SLOWDOWN)/100.0, RIGHT_MOTOR);                           // Inner wheel
            }
        }

    }
    else {  // not running -> stop driving
        TIMER_A0_PWM_DutyCycle(0.0, LEFT_MOTOR);
        TIMER_A0_PWM_DutyCycle(0.0, RIGHT_MOTOR);
    }

    return new_speed;
}

/**
 * @brief Reads camera data, smooths it, finds max and min slope indexes and values
 * @param raw_camera_data raw camera data
 * @param smoothed_camera_data smoothed camera data
 * @return stats containing max and min indexes and values of the slope
 */
line_stats_t parseCameraData(uint16_t* raw_camera_data, uint16_t* smoothed_camera_data){
    line_stats_t line_stats;
    
    if (g_sendData == TRUE){
        LED1_On();

        MovingAverage(raw_camera_data, smoothed_camera_data, &line_stats); // smooth raw data
        slope_finder(smoothed_camera_data, &line_stats);   // get values for stats using derivative

        // render camera data onto the OLED display
        #ifdef USE_OLED
            DisableInterrupts();
            OLED_display_clear();
            OLED_DisplayCameraData(smoothed_camera_data);
            EnableInterrupts();
        #endif
        
        LED1_Off();
        g_sendData = FALSE;
    }
    return line_stats;
}

/**
 * @brief checks if the car is off the track
 * @param camera_max maximum value of camera data
 * @return TRUE if off track
 */
BOOLEAN isOffTrack(uint16_t camera_max){
    if ((10 < camera_max) && (camera_max < 6500)){  // if max val is too low (darkness/carpet)
        return TRUE;    // car is off track
    }
    else{
        return FALSE; // car is not off track
    }
}

/**
 * @brief initializes LEDs, UART, OLED, Steering, and Driving
 */
void init(void){
    DisableInterrupts();
    LED1_Init();
    LED2_Init();
    InitSwitches();
    INIT_Camera();
    initSteering();
    initDriving();

    #ifdef USE_UART
        uart0_init();
        uart2_init();
    #endif
    #ifdef USE_OLED
        OLED_Init();
        OLED_display_on();
        OLED_display_clear();
        OLED_display_on();
    #endif
}

/*
    TODO
    Notes from Beato
    - Could go faster (can't it always lol)
    - Convert error_history [] params to pointer
        - May explain why only term[0] gets through
*/
int main(void){
    line_stats_t line_statistics;   // stats of camera data
    int track_loss_counter = 0;     // off track counter
    double servo_position;          // current position of servo
    double motor_speed = 20.0;

    // Set PID variables to recommended starting points from the Control Systems lecture slides
    pid_values_t steering_pid = {0.13, 0.05, 0.0};
    pid_values_t driving_pid = {0.1, 0.05, 0.0};

    /* Initializations */
    init();

    /* Begin Infinite Loop */
    EnableInterrupts();
    running = FALSE;
    /**
     * On switch 1 press: cycle through race modes (Red, Green, Blue)
     *
     * When switch 2 is pressed,
     * turn running on and select that mode
     * turn LED off
     */

    enum raceMode{Jog, Run, Sprint} raceMode;
    raceMode = Sprint;  //  Jog will show up as first race mode

    for (;;){
        if (Switch1_Pressed()){
            raceMode++;
            switch (raceMode) {
                case Jog:
                    LED2_Red();
                    break;
                case Run:
                    LED2_Green();
                    break;
                case Sprint:
                    LED2_Blue();
                    break;
                default:
                    LED2_Off();
                    break;
            }
        }
        if (Switch2_Pressed()){
            LED2_Off();
            running = TRUE;
            // TODO: assign the values related to the race mode here
            //  might be better to handle that above in the switch statement??
            break;
        }
    }

    for (;;){


        
        #ifdef USE_UART
            //if (uart2_dataAvailable() == TRUE)
            //parseUartCmd(&steering_pid, &driving_pid);
        #endif


        /* Read camera data */
        line_statistics = parseCameraData(line, smoothed_line);

        /* Adjust steering direction */
        servo_position = adjustSteering(line_statistics, steering_pid);
        
        /* Adjust DC Motor speed */
        motor_speed = adjustDriving(line_statistics, driving_pid, motor_speed);

        #ifdef USE_UART
            //sprintf(uart_buffer, "servo: %g;  left line idx:  %d;  right line idx:  %d;\n\r", servo_position, line_statistics.left_slope_index, line_statistics.right_slope_index);
            sprintf(uart_buffer, "%g   %g   %g    %g\r\n", DRIVING_ERROR_HISTORY[0], DRIVING_ERROR_HISTORY[1], DRIVING_ERROR_HISTORY[2], motor_speed);
            uart0_put(uart_buffer);
            //uart2_put(uart_buffer);
        #endif

        /* Check for track loss or intersection */
        if (isOffTrack(line_statistics.max))
            track_loss_counter += 1;    //  if off track, increment counter
        else
            track_loss_counter = 0;     //  if on track, reset counter to zero

        /* Carpet detection reached limit */
        if (track_loss_counter > TRACK_LOSS_LIMIT){ running = FALSE; }

		/* Do a small delay */
		msdelay(10);
    }
}
