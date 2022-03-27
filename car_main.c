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
//#define USE_UART
//#define TEST_OLED

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

double adjustSteering(int degree, double servo_position){
    // TODO: For now, we are changing the steering angle at a constant rate
    double constant_rate = 0.005;

    // TODO: Double check that the left is the right left, I can't remember which is which
    // left, center, right => .05, .075, .1 => 1ms, 1.5ms, 2ms


    /** Need to add logic that follows some sort of dampening scheme to get to the center of the track
     * as fast as possible without turning too far
     *
     * also if calculated degree is withing a given tolerance, it should keep going straight
     * until it falls out of the tolerance and then it will correct the steering. This should prevent
     * the car from constantly turning
     */

    // 0 = high right avg (turn more right)
    if (degree == 1){
        if(servo_position < CENTER_POSITION){ servo_position = CENTER_POSITION; }
        else{
            servo_position = servo_position + constant_rate;
            if ( servo_position > RIGHT_POSITION){ servo_position = RIGHT_POSITION; }
        }

    }

    // 1 = high left avg (turn more left)
    else if (degree == -1){
        if(servo_position > CENTER_POSITION){ servo_position = CENTER_POSITION; }
        else{
            servo_position = servo_position - constant_rate;
            if ( servo_position < LEFT_POSITION){ servo_position = LEFT_POSITION; }
        }

    }

    else {
        servo_position = CENTER_POSITION;
    }

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
            TIMER_A0_PWM_DutyCycle(((3*SPEED)/4)/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(SPEED/100.0, RIGHT_MOTOR);

        }
        else if ( servo_position > CENTER_POSITION ) {
            TIMER_A0_PWM_DutyCycle(SPEED/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(((3*SPEED)/4)/100.0, RIGHT_MOTOR);

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

int parseCameraData(uint16_t* raw_camera_data, uint16_t* avg_line_data){
    int max;
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
        
        LED1_Off();
        g_sendData = FALSE;
    }

    return max;
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
    int camera_line_max = 0;
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
    LED2_Green();

    // TODO: Only run loop for a short period of time as a safety check at first
    for(;;){

        camera_line_max = parseCameraData(line, avg_line);
        direction = determine_direction(avg_line);

        // Turn the servo motor
        servo_position = adjustSteering(direction, servo_position);

        #ifdef USE_UART
            sprintf(uart_buffer, "left: %u;    right: %u;   dir: %d;    max: %u\n\r", avg_line[0], avg_line[64], direction, camera_line_max);
            uart2_put(uart_buffer);
            uart0_put(uart_buffer);
//        sprintf(uart_buffer, "servo position: %g\n\r", servo_position);
//        uart2_put(uart_buffer);
        #endif
        
        // Set the speed the DC motors should spin
        adjustDriving(servo_position);

        if ((10 < camera_line_max) && (camera_line_max < 6500)){
            running = FALSE;
            LED2_Off();
        }

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
