/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
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

#define USE_OLED
#define USE_UART
//#define TEST_OLED

/* Servo Positions */
#define CENTER_POSITION 0.075
#define LEFT_POSITION   0.05
#define SHARP_LEFT      0.055
#define SLIGHT_LEFT     0.065
#define RIGHT_POSITION  0.1
#define SHARP_RIGHT     0.095
#define SLIGHT_RIGHT    0.085

#define CENTER_LEFT  18
#define LEFT_ZONE1   38     // CENTER_LEFT < left_index < LEFT_ZONE1 : Slight right
#define LEFT_ZONE2   58     //  LEFT_ZONE1 < left_index < LEFT_ZONE2 : Sharp right
#define CENTER_RIGHT 110
#define RIGHT_ZONE1  90
#define RIGHT_ZONE2  70

/* Slope Index Thresholds */
#define LOW_LEFT_THRESH  30
#define LOW_RIGHT_THRESH (128-30)

#define MIDPOINT(L_IDX,R_IDX) (((L_IDX) + (R_IDX))/2)

/* DC Motor Settings */
// 3 and 4 motor goes fwd
// 2 and 3 left
// 1 and 4 is right
#define SPEED 21.0  // start out testing very slow
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4
#define HALF_DUTY_CYCLE(dc) ((dc)/200.0)
#define QUARTER_DUTY_CYCLE(dc) ((dc)/400.0)

/* Track Loss Limit */
#define TRACK_LOSS_LIMIT 3

// line stores the current array of camera data
extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
uint16_t line[128];             // raw
uint16_t smoothed_line[128];    // 5-point average of raw data
uint16_t steering_array[128];         
int slope_results[4];
BOOLEAN g_sendData;
BOOLEAN running = TRUE;

/**
 * @brief simple delay function
 */
void myDelay(void){
	volatile int j;
	for (j = 0; j < 800000; j++){}
}

/**
 * @brief msdelay function from Lab 5
 */
void msdelay(int delay){
    int i,j;
    for(i=0;i<delay;i++)
        for(j=0;j<16000;j++);
}


void initSteering(void){
    // Setup steering to be centered
    // PWM -> f = 1kHz, T = 20ms, center = 1.5ms
    TIMER_A2_PWM_Init(CalcPeriodFromFrequency(1000.0), CENTER_POSITION, 1);
}

/*

Tuning

left threshold: 15
sharp left:

right: 95
sharp right:

*/

double adjustSteering(int degree){
    double servo_position = CENTER_POSITION;
    uint16_t left_amt, right_amt;
    uint16_t difference = 0;
    uint16_t threshold = 10000;
    int min_slope, max_slope;
    int left_line_index, right_line_index;
    int track_midpoint_idx = 64;
    
    double rl_ratio = 0;

//    left_amt = steering_array[0];
//    right_amt = steering_array[64];
//    difference = abs(right_amt - left_amt);

    right_line_index = slope_results[0];
    left_line_index = slope_results[1];
    min_slope = slope_results[2];
    max_slope = slope_results[3];
    
    if (right_line_index > left_line_index){    //  center, slight right, slight left, edge case
        rl_ratio = right_amt/left_amt;
        if ((rl_ratio > 0) && (rl_ratio < 2)){
            track_midpoint_idx = MIDPOINT(left_line_index, right_line_index);   
            if ((track_midpoint_idx >= 60) && (track_midpoint_idx <= 68)){  // drive straight
                servo_position = CENTER_POSITION;
                LED2_Green();
            }
            else{
                if(track_midpoint_idx > 68){    //  slight right
                    servo_position = SLIGHT_RIGHT;
                    LED2_Magenta();
                }
                else if (track_midpoint_idx < 60){  //  slight left
                    servo_position = SLIGHT_LEFT;
                    LED2_Cyan();
                }
            }
             
        }
    }
    else if(left_line_index > right_line_index){
        if(left_amt > right_amt){   //  big right 
            servo_position = SHARP_RIGHT;
            LED2_Red();
        }
        else if (right_amt > left_amt){ //  big left
            servo_position = SHARP_LEFT;
            LED2_Blue();
        }
    }

    
    
    
    
//    if (left_line_index < CENTER_LEFT){
//        if (CENTER_RIGHT < right_line_index){
//            servo_position = CENTER_POSITION;
//            LED2_Green();
//        }
//        else if ((RIGHT_ZONE1 <= right_line_index) && (right_line_index <= CENTER_RIGHT)){
//            servo_position = SLIGHT_LEFT;
//            LED2_Cyan();
//        }
//        else if (left_line_index < right_line_index){
//            if ((-1*min_slope) < max_slope){
//                servo_position = SLIGHT_RIGHT;
//                LED2_Magenta();
//            }
//            else {
//                servo_position = SHARP_LEFT;
//                LED2_Blue();
//            }
//        }
//    }
//    else if ((CENTER_LEFT <= left_line_index) && (left_line_index <= LEFT_ZONE1)){
//        servo_position = SLIGHT_RIGHT;
//        LED2_Magenta();
//    }
//    else if (right_line_index < left_line_index){
//        if (max_slope < (-1*min_slope)){
//            servo_position = SLIGHT_LEFT;
//            LED2_Cyan();
//        }
//        else {
//            servo_position = SHARP_RIGHT;
//            LED2_Red();
//        }
//    }
//    else {
//        servo_position = SHARP_LEFT;
//        LED2_Blue();
//    }

    TIMER_A2_PWM_DutyCycle(servo_position, 1);
    return servo_position;
}

void initDriving(void){
    uint16_t period;
    double freq = 10000.0;  //  Frequency = 10 kHz

    //  EN_A and EN_B are tied to H-Bridge enable pins on the motor shield
    //  M1EN -> P3.6
    //  M2EN -> P3.7
    //  TODO: Set to GPIO Mode and set to logic '1'
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

    /**
     * M1A -> P2.4 -> TA0.1
     * M1B -> P2.4 -> TA0.2
     * M2A -> P2.4 -> TA0.3
     * M2B -> P2.4 -> TA0.4
     */

    period = (uint16_t) CalcPeriodFromFrequency(freq);
    TIMER_A0_PWM_Init(period, 0.0, 1);	// M1A -> P2.4
    TIMER_A0_PWM_Init(period, 0.0, 2);	// M1B -> P2.5
    TIMER_A0_PWM_Init(period, 0.0, 3);	// M2A -> P2.6
    TIMER_A0_PWM_Init(period, 0.0, 4);	// M2B -> P2.7
}

void adjustDriving(double servo_position){

    // if servo position is within .05 of center, keep driving at same speed
    double right_limit = CENTER_POSITION + .0025;
    double left_limit = CENTER_POSITION - .0025;

    if (running){

        if (servo_position < CENTER_POSITION){
            TIMER_A0_PWM_DutyCycle(((5.0*SPEED)/6.0)/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle((2.0+SPEED)/100.0, RIGHT_MOTOR);

        }
        else if ( servo_position > CENTER_POSITION ) {
            TIMER_A0_PWM_DutyCycle((2.0+SPEED)/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(((5.0*SPEED)/6.0)/100.0, RIGHT_MOTOR);

        }
        else{
            TIMER_A0_PWM_DutyCycle(SPEED/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(SPEED/100.0, RIGHT_MOTOR);
        }

    }
    else {
        TIMER_A0_PWM_DutyCycle(0.0, LEFT_MOTOR);
        TIMER_A0_PWM_DutyCycle(0.0, RIGHT_MOTOR);
    }

    // if(running){
    //     if ((servo_position <= right_limit) || (servo_position >= left_limit)){
    //         //  motor 1 left => P2.4 & P2.5
    //         TIMER_A0_PWM_DutyCycle(duty_cycle/100.0, LEFT_MOTOR);  //  motor 1 left

    //         //  motor 2 right => P2.6 & P2.7
    //         TIMER_A0_PWM_DutyCycle(duty_cycle/100.0, RIGHT_MOTOR);  //  motor 2 right
    //     }
    //         // if servo position is left (less than center)
    //         // trying to turn left, slow down left wheel
    //     else if (servo_position < CENTER_POSITION){
    //         // Set speed of right wheel
    //         TIMER_A0_PWM_DutyCycle(duty_cycle/100.0, RIGHT_MOTOR);  //  motor 2 right

    //         // if steering is more than half the distance between center and left
    //         // cut speed of left by duty_cycle/4
    //         if (servo_position < CENTER_POSITION - half_turn){
    //             TIMER_A0_PWM_DutyCycle(QUARTER_DUTY_CYCLE(duty_cycle), LEFT_MOTOR);  //  motor 1 left
    //         }

    //             // else cut speed of left by duty_cycle/2
    //         else{
    //             TIMER_A0_PWM_DutyCycle(HALF_DUTY_CYCLE(duty_cycle), LEFT_MOTOR);  //  motor 1 left
    //         }
    //     }

    //         // if servo position is right (more than center)
    //         // trying to turn right, slow down right wheel
    //     else if (servo_position > CENTER_POSITION){
    //         // Set speed of left wheel
    //         TIMER_A0_PWM_DutyCycle(duty_cycle/100.0, 1);  //  motor 1 left

    //         // if steering is more than half the distance between center and right
    //         // cut speed of right by duty_cycle/4
    //         if (servo_position > CENTER_POSITION + half_turn){
    //             TIMER_A0_PWM_DutyCycle(QUARTER_DUTY_CYCLE(duty_cycle), RIGHT_MOTOR);  //  motor 2 right
    //         }

    //             // else cut speed of right by duty_cycle/2
    //         else{
    //             TIMER_A0_PWM_DutyCycle(HALF_DUTY_CYCLE(duty_cycle), RIGHT_MOTOR);  //  motor 2 right
    //         }
    //     }
    // }
    // else{
    //     // set all motors to zero (stopped)
    //     TIMER_A0_PWM_DutyCycle(0.0, 1);  //  motor 1 left

    //     TIMER_A0_PWM_DutyCycle(0.0, 3);  //  motor 2 right
    // }
}

uint16_t parseCameraData(uint16_t* raw_camera_data, uint16_t* smoothed_line, uint16_t* avg_line_data){
    uint16_t max;
    
    if (g_sendData == TRUE){
        LED1_On();

        max = MovingAverage(raw_camera_data, smoothed_line);
        slope_finder(smoothed_line, slope_results);

        // render camera data onto the OLED display
        #ifdef USE_OLED
            DisableInterrupts();
            OLED_display_clear();
            OLED_DisplayCameraData(smoothed_line);
            EnableInterrupts();
        #endif
        
        LED1_Off();
        g_sendData = FALSE;
    }

    return max;
}

int CheckForTrackLoss(uint16_t camera_max, int counter){
    int retVal = counter;

    if ((10 < camera_max) && (camera_max < 6500)){
        retVal = counter + 1;
    }

    if (TRACK_LOSS_LIMIT <= retVal){
        running = FALSE;
    }

    return retVal;
}

/**
 * @brief initializes LED1, LED2, UART, OLED, and PWM
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
        #ifdef TEST_OLED            // Cascaded ifdef to avoid initializing OLED screen twice
            OLED_Init();
            OLED_display_on();
            OLED_display_clear();
            OLED_display_on();
        #endif
    #endif
}



int main(void){
    int direction = 0;  // 1 = high left avg (turn left) 0 = high right avg (turn right)
    uint16_t camera_line_max = 0;
    int track_loss_counter = 0;
    double servo_position = 0.075;

    #ifdef USE_UART
        char uart_buffer [20];
    #endif

    /* Initializations */
    init();
    
    /* Test OLED Display*/
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

    // TODO: Only run loop for a short period of time as a safety check at first
    for(;;){

        camera_line_max = parseCameraData(line, smoothed_line, steering_array);
        direction = determine_direction(steering_array);

        // Turn the servo motor
        servo_position = adjustSteering(direction);

        #ifdef USE_UART
            sprintf(uart_buffer, "servo: %g;  left line idx:  %d;  right line idx:  %d;\n\r", servo_position, slope_results[1], slope_results[0]);
            uart2_put(uart_buffer);
            uart0_put(uart_buffer);
        #endif
        
        // Set the speed the DC motors should spin
        adjustDriving(servo_position);

        // Check for track loss or intersection
        //track_loss_counter = CheckForTrackLoss(camera_line_max, track_loss_counter);
        if ((10 < camera_line_max) && (camera_line_max < 6500))
            running = FALSE;

        // Reset local variables
        camera_line_max = 0;
        
		// Do a small delay
		msdelay(100);
    }


    // TODO: Use this formatting instead for a clear, understandable main loop
//    for(;;){
//
//        // Read the line scan camera data
//        readCameraData(line);
//
//
//        // Smooth and filter the raw data
//
//
//        // Determine the degree to turn the servo
//        direction = determine_direction(avg_line);
//
//
//        // Turn the servo motor
//        servo_position = adjustSteering(direction, servo_position);
//
//        // Set the speed the DC motors should spin
//        adjustDriving(servo_position, running);
//
//        // do a small delay
//        myDelay();
//    }
}
