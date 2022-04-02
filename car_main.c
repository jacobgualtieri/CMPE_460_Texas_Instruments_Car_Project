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
#include "Steering_PID.h"

/* Testing and debugging */
//#define USE_OLED
//#define USE_UART
//#define TEST_OLED

/* Servo Positions */
#define CENTER_POSITION   0.075
#define SHARP_LEFT        0.05  //  .005 from slight left
#define SLIGHT_LEFT       0.059 //  .01 from center
#define SHARP_RIGHT       0.091 //  .005 from slight right
#define SLIGHT_RIGHT      0.085 //  .01 from center
#define ADJUSTMENT_THRESH 7800

/* Directional Thresholds */
#define CENTER_LEFT_IDX  58
#define CENTER_RIGHT_IDX 76
#define RIGHT_IDX_OFFSET 15

/* Speed Settings */
#define STRAIGHTS_SPEED 23.5
#define SPEED           23.5

/* DC Motor Settings */
// 3 and 4 motor goes fwd
// FORWARD: LEFT <= 3, RIGHT <= 4
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

/* Track Loss Limit */
#define TRACK_LOSS_LIMIT 5  // Stop limit if off track

// line stores the current array of camera data
extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];

/* Servo Position History Array */
double ERROR_HISTORY[HISTORY_LENGTH];

uint16_t line[128];             // raw camera data
uint16_t smoothed_line[128];    // 5-point average of raw data
BOOLEAN g_sendData;             // TRUE if camera data is ready to read
BOOLEAN running = TRUE;         // Driving control variable

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
    for (i = 0; i < HISTORY_LENGTH; i++){ ERROR_HISTORY[i] = 0.0; }

    // Setup steering to be centered
    // PWM -> f = 1kHz, T = 20ms, center = 1.5ms
    TIMER_A2_PWM_Init(CalcPeriodFromFrequency(1000.0), CENTER_POSITION, 1);
}

/*
// TODO: Do we still need this?
Tuning

left threshold: 15
sharp left:

right: 95
sharp right:

*/

/**
 * @brief Adjusts steering based on camera input
 *
 * @param line_stats contains values and indexes of the max and min
 * values of the derivative (slope) of the input data
 * @param current_servo_position current position of the servo
 * @return new servo position
 */
double adjustSteering(line_stats_t line_stats, double current_servo_position){
    double servo_position = current_servo_position;
    uint16_t left_amt, right_amt;
    int left_line_index, right_line_index;
    int track_midpoint_idx;

    right_line_index = line_stats.right_slope_index - RIGHT_IDX_OFFSET;
    left_line_index = line_stats.left_slope_index;
    right_amt = -1 * line_stats.right_slope_amount;
    left_amt = line_stats.left_slope_amount;
    
    if (6800 < line_stats.min){     // If the lowest value detected is fairly high, we can go straight ahead
        servo_position = CENTER_POSITION;
    }
    else if (line_stats.max > ADJUSTMENT_THRESH){
        if (right_line_index > left_line_index){    //  Standard Cases: center, slight right, slight left
            track_midpoint_idx = MIDPOINT(left_line_index, right_line_index);   
            if ((track_midpoint_idx >= CENTER_LEFT_IDX) && (track_midpoint_idx <= CENTER_RIGHT_IDX)){  // drive straight
                servo_position = CENTER_POSITION;
                LED2_Green();
            }
            else{
                if(track_midpoint_idx > CENTER_RIGHT_IDX){      //  slight left
                    servo_position = SLIGHT_LEFT;
                    LED2_Magenta();
                }
                else if (track_midpoint_idx < CENTER_LEFT_IDX){ //  slight right
                    servo_position = SLIGHT_RIGHT;
                    LED2_Cyan();
                }
            }
        }
        else {      // Directional Error Cases
            if (left_amt > right_amt){  // sharp left
                servo_position = SHARP_LEFT;
                LED2_Red();
            }
            else {                      // sharp right
                servo_position = SHARP_RIGHT;
                LED2_Blue();
            }
        }
    }
    TIMER_A2_PWM_DutyCycle(servo_position, 1);  // set new servo position
    return servo_position;
}

/**
 * @brief Initializes DC motors and enable pins on motor driver board
 */
void initDriving(void){
    uint16_t period;
    double freq = 10000.0;  //  Frequency = 10 kHz

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
 * @param servo_position position of the servo
 */
void adjustDriving(double servo_position){
    if (running){
        if (servo_position == SHARP_LEFT){
            TIMER_A0_PWM_DutyCycle((SPEED-4.0)/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle((9.0+SPEED)/100.0, RIGHT_MOTOR);

        }
        else if ( servo_position == SLIGHT_LEFT ){
            TIMER_A0_PWM_DutyCycle((SPEED-2.0)/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle((3.0+SPEED)/100.0, RIGHT_MOTOR);
        }
        else if ( servo_position == SHARP_RIGHT ) {
            TIMER_A0_PWM_DutyCycle((4.0+SPEED)/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle((SPEED-4.0)/100.0, RIGHT_MOTOR);

        }
        else{   // Possible cases: Slight right, Straight
            TIMER_A0_PWM_DutyCycle(STRAIGHTS_SPEED/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(STRAIGHTS_SPEED/100.0, RIGHT_MOTOR);
        }
    }
    else {  // not running -> stop driving
        TIMER_A0_PWM_DutyCycle(0.0, LEFT_MOTOR);
        TIMER_A0_PWM_DutyCycle(0.0, RIGHT_MOTOR);
    }
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
        return TRUE;
    }
    else{
        return FALSE;
    }
}

/**
 * @brief initializes LEDs, UART, OLED, Steering, and Driving
 */
void init(void){
    DisableInterrupts();
    LED1_Init();
    LED2_Init();
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
    #else
        #ifdef TEST_OLED            // Cascaded if-def to avoid initializing OLED screen twice
            OLED_Init();
            OLED_display_on();
            OLED_display_clear();
            OLED_display_on();
        #endif
    #endif
}

void PrintPIDValues(pid_values steer, pid_values drive){
    PrintSteeringValues(steer);
    PrintDrivingValues(drive);
}

int main(void){
    line_stats_t line_statistics;   // stats of camera data
    int track_loss_counter = 0;     // off track counter
    double servo_position = 0.075;  // current position of servo

    // TODO: here are the PID variables
    pid_values steering_pid = {0.1, 0.2, 0.3};
    pid_values driving_pid = {0.1, 0.2, 0.3};

    #ifdef USE_UART
        char uart_buffer [20];
    #endif

    /* Initializations */
    init();

    /* Test OLED Display */
    #ifdef TEST_OLED
        OLED_draw_line(1, 1, (unsigned char *)"Hello World");   // casting strings to (unsigned char *) bc keil is stupid
        OLED_draw_line(2, 2, (unsigned char *)"How are you?");
        OLED_draw_line(3, 3, (unsigned char *)"Goodbye");
        OLED_write_display(OLED_TEXT_ARR);
        msdelay(1000);
        for(j = 0; j < 1024; j++){ OLED_TEXT_ARR[j] = 0; }
        OLED_display_clear();
    #endif

    /* Begin Infinite Loop */
    EnableInterrupts();
    running = TRUE;

    for(;;){

        /* Check if a message was sent from UART2 */
        // TODO: Add logic here
        // message = getfromUART2();
        // updatePID(message);


        // TODO: add function to split the string into tokens,
        //  get the number of tokens
        //  if the num == 4 then read the first token and call the steer or
        //  drive update PID values funciton
        //  if the num == 1 then print out the help command or print all PID values



        // if the message sent from uart2 was "steer kp ki kd"
        // Update current PID steering variables
        PrintSteeringValues(steering_pid);


        // else if "drive kp ki kd"
        // Update current PID driving variables
        // print out new PID values
        PrintDrivingValues(driving_pid);

        // else if "help"
        // print "steer kp ki kd" OR "drive kp ki kd" OR "help" OR "values"

        // else if "values"
        // print out current variable values for steering and driving
        // "steering values: Kp = 1, Ki = 2, Kd = 3"
        // "driving values: Kp = 1, Ki = 2, Kd = 3"


        /* Read camera data */
        line_statistics = parseCameraData(line, smoothed_line);

        /* Turn the servo motor */
        servo_position = adjustSteering(line_statistics, servo_position);

        #ifdef USE_UART
            sprintf(uart_buffer, "servo: %g;  left line idx:  %d;  right line idx:  %d;\n\r", servo_position, line_statistics.left_slope_index, line_statistics.right_slope_index);
            uart2_put(uart_buffer);
            uart0_put(uart_buffer);
        #endif

        /* Set the speed the DC motors should spin */
        adjustDriving(servo_position);

        /* Check for track loss or intersection */
        if (isOffTrack(line_statistics.max)) {
            track_loss_counter += 1;    //  if off track increment counter
        }
        else {
            track_loss_counter = 0; //  if on track, reset counter to zero
        }

        /* carpet detection reached count limit */
        if (track_loss_counter > TRACK_LOSS_LIMIT){ running = FALSE; }

		/* Do a small delay */
		msdelay(10);
    }
}
