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

/* UART */
#define UART2_RX_BUFFER_LENGTH 20

/* Servo Positions */
#define CENTER_POSITION   0.075
#define SHARP_RIGHT        0.05  //  .005 from slight left
#define SLIGHT_RIGHT       0.059 //  .01 from center
#define SHARP_LEFT       0.11 //  .005 from slight right
#define SLIGHT_LEFT      0.085 //  .01 from center
#define ADJUSTMENT_THRESH 7800

/* Directional Thresholds */
#define CENTER_LEFT_IDX  58
#define CENTER_RIGHT_IDX 76
#define RIGHT_IDX_OFFSET 2

/* Speed Settings */
#define STRAIGHTS_SPEED     40.0
#define CORNERING_SPEED     30.0
#define INNER_WHEEL_SLOWDOWN 5.0

/* DC Motor Settings */
// 3 and 4 motor goes fwd
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
BOOLEAN running = FALSE;         // Driving control variable
BOOLEAN enableSpeedPID = FALSE;

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

/*
void parseUartCmd(pid_values_t* steering_settings_ptr, pid_values_t* driving_settings_ptr){

    char user_cmd_char = 0;
    float new_kp = 0.0;
    float new_ki = 0.0;
    float new_kd = 0.0;

    uart2_get(uart_rx_buffer, UART2_RX_BUFFER_LENGTH);
    // sscanf(uart_rx_buffer, "%c %f %f %f", &user_cmd_char, &new_kp, &new_ki, &new_kd);
    uart0_put(uart_rx_buffer);
    uart2_put(uart_rx_buffer);
    
//    if ((user_cmd_char == 'd') || (user_cmd_char == 'D')){
//        UPDATE_PID(*driving_settings_ptr, new_kp, new_ki, new_kd);
//        PrintDrivingValues(*driving_settings_ptr);
//    }
//    else if ((user_cmd_char == 's') || (user_cmd_char == 'S')){
//        UPDATE_PID(*steering_settings_ptr, new_kp, new_ki, new_kd);
//        PrintSteeringValues(*steering_settings_ptr);
//    }
//    else if ((user_cmd_char == 'h') || (user_cmd_char == 'H')){
//        uart0_put("'s kp ki kd' OR 'd kp ki kd' OR 'h' OR 'v'\r\n");
//        uart2_put("'s kp ki kd' OR 'd kp ki kd' OR 'h' OR 'v'\r\n");
//    }
//    else if ((user_cmd_char == 'v') || (user_cmd_char == 'V')){
//        PrintSteeringValues(*steering_settings_ptr);
//        PrintDrivingValues(*driving_settings_ptr);
//    }
//    else if ((user_cmd_char == 'g') || (user_cmd_char == 'G')){
//        running = TRUE;
//        uart0_put("Starting car...\r\n");
//        uart2_put("Starting car...\r\n");
//    }
//    else if ((user_cmd_char == 'x') || (user_cmd_char == 'X')){
//        running = FALSE;
//        uart0_put("Stopping car...\r\n");
//        uart2_put("Stopping car...\r\n");
//    }
}
*/

/**
 * @brief Adjusts steering based on camera input
 *
 * @param line_stats contains values and indexes of the max and min
 * values of the derivative (slope) of the input data
 * @param current_servo_position current position of the servo
 * @return new servo position
 */
double adjustSteering(line_stats_t line_stats, pid_values_t pid_params, double current_servo_position){
    double servo_position = current_servo_position;
    uint16_t left_amt, right_amt;
    int left_line_index, right_line_index;
    int track_midpoint_idx = 64;
    int delta = 0;

    right_line_index = line_stats.right_slope_index - RIGHT_IDX_OFFSET;
    left_line_index = line_stats.left_slope_index;
    right_amt = -1 * line_stats.right_slope_amount;
    left_amt = line_stats.left_slope_amount;
    
    //servo_position = MIDPOINT()/64 * CENTER_POSITION;
    
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
    
    
    // Turning Right
    servo_position = ((double)delta / 64.0) * CENTER_POSITION;
    
    // Turning Left
    if (track_midpoint_idx < 64){
        servo_position += (double)CENTER_POSITION;
    }
    
    // Prevent control loop from exceeding servo range
    if (servo_position < SHARP_RIGHT){
        servo_position = SHARP_RIGHT;
    }
    else if (SHARP_LEFT < servo_position){
        servo_position = SHARP_LEFT;
    }
    
//    
//    if (6800 < line_stats.min){     // If the lowest value detected is fairly high, we can go straight ahead
//        servo_position = CENTER_POSITION;
//    }
//    else if (line_stats.max > ADJUSTMENT_THRESH){
//        if (right_line_index > left_line_index){    //  Standard Cases: center, slight right, slight left
//            track_midpoint_idx = MIDPOINT(left_line_index, right_line_index);   
//            if ((track_midpoint_idx >= CENTER_LEFT_IDX) && (track_midpoint_idx <= CENTER_RIGHT_IDX)){  // drive straight
//                servo_position = CENTER_POSITION;
//            }
//            else{
//                if(track_midpoint_idx > CENTER_RIGHT_IDX){      //  slight left
//                    servo_position = SLIGHT_RIGHT;
//                }
//                else if (track_midpoint_idx < CENTER_LEFT_IDX){ //  slight right
//                    servo_position = SLIGHT_LEFT;
//                }
//            }
//        }
//        else {      // Directional Error Cases
//            if (left_amt > right_amt){  // sharp left
//                servo_position = SHARP_RIGHT;
//            }
//            else {                      // sharp right
//                servo_position = SHARP_LEFT;
//            }
//        }
//    }
    
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
 * @param servo_position position of the servo
 */
double adjustDriving(line_stats_t line_stats, pid_values_t pid_params, double current_speed, double servo_position){
    uint16_t left_amt, right_amt;
    int left_line_index, right_line_index;
    int track_midpoint_idx = 64;
    int delta = 0;
    double new_speed = 0.0;

    if (running){

        // PID-Based Speed Control
        if (enableSpeedPID){
            right_line_index = line_stats.right_slope_index - RIGHT_IDX_OFFSET;
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

            if (delta < 12){
                new_speed = DrivingPID(pid_params, STRAIGHTS_SPEED, current_speed);
                
                TIMER_A0_PWM_DutyCycle(new_speed/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle(new_speed/100.0, RIGHT_MOTOR);
            }
            else {
                new_speed = DrivingPID(pid_params, CORNERING_SPEED, current_speed);

                if (track_midpoint_idx < 64){                                           // Making a left turn
                    TIMER_A0_PWM_DutyCycle((new_speed - INNER_WHEEL_SLOWDOWN)/100.0, LEFT_MOTOR);                            // Inner wheel
                    TIMER_A0_PWM_DutyCycle((new_speed)/100.0, RIGHT_MOTOR);   // Outer wheel
                }
                else {                                                                  // Making a right turn
                    TIMER_A0_PWM_DutyCycle((new_speed)/100.0, LEFT_MOTOR);    // Outer wheel
                    TIMER_A0_PWM_DutyCycle((new_speed - INNER_WHEEL_SLOWDOWN)/100.0, RIGHT_MOTOR);                           // Inner wheel
                }
            }   

        }

        // Non-PID Speed Control
        else {
            if (servo_position == SHARP_LEFT){
                TIMER_A0_PWM_DutyCycle((STRAIGHTS_SPEED-4.0)/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle((9.0+STRAIGHTS_SPEED)/100.0, RIGHT_MOTOR);
            }
            // else if ( servo_position == SLIGHT_LEFT ){
            //     TIMER_A0_PWM_DutyCycle((STRAIGHTS_SPEED-2.0)/100.0, LEFT_MOTOR);
            //     TIMER_A0_PWM_DutyCycle((3.0+STRAIGHTS_SPEED)/100.0, RIGHT_MOTOR);
            // }
            else if ( servo_position == SHARP_RIGHT ) {
                TIMER_A0_PWM_DutyCycle((4.0+STRAIGHTS_SPEED)/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle((STRAIGHTS_SPEED-4.0)/100.0, RIGHT_MOTOR);
            }
            else {   // Possible cases: Slight right, Slight left, Straight
                TIMER_A0_PWM_DutyCycle(STRAIGHTS_SPEED/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle(STRAIGHTS_SPEED/100.0, RIGHT_MOTOR);
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

void PrintPIDValues(pid_values_t steer, pid_values_t drive){
    PrintSteeringValues(steer);
    PrintDrivingValues(drive);
}

int main(void){
    line_stats_t line_statistics;   // stats of camera data
    int track_loss_counter = 0;     // off track counter
    double servo_position = 0.075;  // current position of servo
    double motor_speed = 20.0;

    // Set PID variables to recommended starting points from the Control Systems lecture slides
    pid_values_t steering_pid = {0.07, 0.0, 0.0};
    pid_values_t driving_pid = {0.3, 0.0, 0.0};

    /* Initializations */
    init();

    /* Begin Infinite Loop */
    EnableInterrupts();
    running = TRUE;
    enableSpeedPID = TRUE;

    for(;;){

        /*
        Check if a message was sent from UART2
        Example format: d 0.5 0.1 0.25
                        S 0.75 0.4 0.3
        */


        // TODO: add function to split the string into tokens,
        //  get the number of tokens
        //  if the num == 4 then read the first token and call the steer or
        //  drive update PID values funciton
        //  if the num == 1 then print out the help command or print all PID values


        // if the message sent from uart2 was "steer kp ki kd"
        // Update current PID steering variables


        // else if "drive kp ki kd"
        // Update current PID driving variables
        // print out new PID values

        // else if "help"
        // print "steer kp ki kd" OR "drive kp ki kd" OR "help" OR "values"

        // else if "values"
        // print out current variable values for steering and driving
        // "steering values: Kp = 1, Ki = 2, Kd = 3"
        // "driving values: Kp = 1, Ki = 2, Kd = 3"
        
        #ifdef USE_UART
            //if (uart2_dataAvailable() == TRUE)
                //parseUartCmd(&steering_pid, &driving_pid);
        #endif


        /* Read camera data */
        line_statistics = parseCameraData(line, smoothed_line);

        /* Turn the servo motor */
        servo_position = adjustSteering(line_statistics, steering_pid, servo_position);
        
        /* Set the speed the DC motors should spin */
        motor_speed = adjustDriving(line_statistics, driving_pid, motor_speed, servo_position);

        #ifdef USE_UART
            //sprintf(uart_buffer, "servo: %g;  left line idx:  %d;  right line idx:  %d;\n\r", servo_position, line_statistics.left_slope_index, line_statistics.right_slope_index);
            sprintf(uart_buffer, "%g   %g   %g    %g\r\n", DRIVING_ERROR_HISTORY[0], DRIVING_ERROR_HISTORY[1], DRIVING_ERROR_HISTORY[2], motor_speed);
            //uart0_put(uart_buffer);
            uart2_put(uart_buffer);
        #endif

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
