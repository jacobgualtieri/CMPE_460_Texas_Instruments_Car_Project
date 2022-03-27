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
#include <math.h>
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
//#define TEST_OLED
uint16_t max;

#define LEFT_POSITION .05
#define RIGHT_POSITION .1
#define CENTER_POSITION .075
#define SPEED 20.0  //  start out testing very slow

#define HALF_DUTY_CYCLE(dc) ((dc)/200.0)
#define QUARTER_DUTY_CYCLE(dc) ((dc)/400.0)

// 3 and 4 motor goes fwd
// 2 and 3 left
// 1 and 4 is right

#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

// line stores the current array of camera data
extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
uint16_t line[128];
uint16_t avg_line[128];
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
    TIMER_A2_PWM_Init(CalcPeriodFromFrequency(1000.0), 0.075, 1);
}

void adjustSteering(double degree){
    double dutycycle_per_degree = (CENTER_POSITION - LEFT_POSITION)/90.0;    // duty cycle per degree
    double duty_cycle;
    // left, center, right => .05, .075, .1 => 1ms, 1.5ms, 2ms

    /** Need to add logic that follows some sort of dampening scheme to get to the center of the track
     * as fast as possible without turning too far
     */

    // A full 90-degree angle will result in either adding or subtracting .025 from CENTER_POSITION
    // which will never reach outside the left and right bounds

    if (degree > 0){    // positive degree = turn right
        duty_cycle = CENTER_POSITION + (degree * dutycycle_per_degree); //  duty cycle will be > CENTER_POSITION
    }

    // redundant code, but easier to see logic for now
    else if (degree < 0){   // negative degree = turn left
        duty_cycle = CENTER_POSITION + (degree * dutycycle_per_degree); //  degree will be negative here
    }

    else {  // zero degree = go straight
        duty_cycle = CENTER_POSITION;
    }

    TIMER_A2_PWM_DutyCycle(duty_cycle, 1);
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

void adjustDriving(double degree){
    if (running){
        if (fabs(degree) < 30){ //  if degree is w/in +- 30 degrees
            TIMER_A0_PWM_DutyCycle(SPEED/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(SPEED/100.0, RIGHT_MOTOR);
        }
        else if (fabs(degree) < 60){    //  if degree is between +- 30 and +- 60 degrees
            if (degree < 0){    //  turning left
                TIMER_A0_PWM_DutyCycle(((3*SPEED)/4)/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle(SPEED/100.0, RIGHT_MOTOR);
            }
            else if (degree > 0){    //  turning right
                TIMER_A0_PWM_DutyCycle(SPEED/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle(((3*SPEED)/4)/100.0, RIGHT_MOTOR);
            }
        }
        else {  //  if degree is between +- 60 and +- 90 degrees
            if (degree < 0){    //  turning left
                TIMER_A0_PWM_DutyCycle((SPEED/2)/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle(SPEED/100.0, RIGHT_MOTOR);
            }
            else if (degree > 0){    //  turning right
                TIMER_A0_PWM_DutyCycle(SPEED/100.0, LEFT_MOTOR);
                TIMER_A0_PWM_DutyCycle((SPEED/2)/100.0, RIGHT_MOTOR);
            }
        }
    }
    else {
        TIMER_A0_PWM_DutyCycle(0.0, LEFT_MOTOR);
        TIMER_A0_PWM_DutyCycle(0.0, RIGHT_MOTOR);
    }
}

void parseCameraData(uint16_t* raw_camera_data, uint16_t* avg_line_data){
    int j = 0;
    if (g_sendData == TRUE){

        LED1_On();
        split_average(raw_camera_data, avg_line_data);

        // render camera data onto the OLED display
        #ifdef USE_OLED
            DisableInterrupts();
            OLED_display_clear();
            OLED_DisplayCameraData(raw_camera_data);
            EnableInterrupts();
        #endif
        
        for(j = 0; j < 128; j++){
            if (max < line[j]){
                max = line[j];
            } else {
                max = max;
            }
        }
        
        if (max < 4096){
            running = FALSE;
        }
        
        LED1_Off();
        g_sendData = FALSE;
    }
}

/**
 * @brief initializes LED1, LED2, UART, OLED, and PWM
 */
void init(void){
    DisableInterrupts();
    LED1_Init();
    //LED2_Init();
    //uart0_init();
    uart2_init();
    INIT_Camera();
    initSteering();
    initDriving();

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
    int degree;  // 0 = straight, + turn right, - turn left
    int j = 0;
    int temp = 0;
    char uart_buffer [20];
    uint64_t camera_sum = 0;

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
    //for (temp = 0; temp < 100; temp++){
    for(;;){

        parseCameraData(line, avg_line);
        degree = determine_direction(avg_line);

        // Turn the servo motor
        adjustSteering(degree);

        sprintf(uart_buffer, "left: %u;    right: %u;   dir: %d;    max: %u\n\r", avg_line[0], avg_line[64], degree, max);
        uart2_put(uart_buffer);
//        sprintf(uart_buffer, "servo position: %g\n\r", servo_position);
//        uart2_put(uart_buffer);
        camera_sum = 0;

        max = 0;
        
        // Set the speed the DC motors should spin
        adjustDriving(degree);
        
		// do a small delay
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
