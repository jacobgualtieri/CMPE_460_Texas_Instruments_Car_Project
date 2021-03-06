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

/* Directional Thresholds */
#define LEFT_TURN_OFFSET 0

/* Speed Settings */
#define STRAIGHTS_SPEED     30.0    //  desired speed in the straight
#define CORNERING_SPEED     27.0    //  desired speed in the corner
#define INNER_WHEEL_SLOWDOWN 6.0    //  decrease factor for inner wheel on turns

/* DC Motor Settings */
// 3 and 4 motor goes' fwd
// FORWARD: LEFT <= 3, RIGHT <= 4
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

/* Track Loss Limit */
#define TRACK_LOSS_LIMIT 12  // Stop limit if off track
#define CARPET_THRESHOLD 5500   // any y value lower than this means the car is off track

#ifdef USE_OLED
    extern unsigned char OLED_clr_data[1024];
    extern unsigned char OLED_TEXT_ARR[1024];
    extern unsigned char OLED_GRAPH_ARR[1024];
#endif

#ifdef USE_UART
    char uart_tx_buffer [20];
    char uart_rx_buffer [UART2_RX_BUFFER_LENGTH];
#endif

/* Servo Position History Array */
double STEERING_ERROR_HISTORY[HISTORY_LENGTH] = {0.0, 0.0, 0.0};
double DRIVING_ERROR_HISTORY[HISTORY_LENGTH] = {0.0, 0.0, 0.0};

/* line stores the current array of camera data */
uint16_t line[128];             // raw camera data
uint16_t smoothed_line[128];    // 5-point average of raw data
BOOLEAN g_sendData;             // TRUE if camera data is ready to read
BOOLEAN running = FALSE;        // Driving control variable

typedef struct speed_settings {
    double straight_speed;
    double corner_speed;
    double inner_wheel_slowdown;
    double outer_wheel_speedup;
    double max_corner_slowdown;
} speed_settings;

enum raceMode {
    Jog = 0,
    Run = 1,
    Sprint = 2
} raceMode;

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
void adjustSteering(center_of_mass_t line_stats, pid_values_t pid_params){
    double servo_position;
    int track_midpoint_idx;
    int delta;

    track_midpoint_idx = line_stats.x;

    delta = 64-track_midpoint_idx;

    if (delta > 0)
        delta += LEFT_TURN_OFFSET;
    
    if (delta == 0)
        servo_position = CENTER_POSITION;
    else
        //servo_position = CENTER_POSITION + (((double)delta / 44.0) * CENTER_POSITION);
        servo_position = CENTER_POSITION + (((double)delta / 48.0) * CENTER_POSITION);
    
    #ifdef USE_UART
        sprintf(uart_tx_buffer, "delta = %d,   servo = %g\n\r", delta, servo_position);
        uart2_put(uart_tx_buffer);
    #endif

    servo_position = SteeringPID(pid_params, CENTER_POSITION, servo_position);
    TIMER_A2_PWM_DutyCycle(servo_position, 1);  // set new servo position
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
double adjustDrivingContinuous(center_of_mass_t line_stats, pid_values_t pid_params, double current_speed, speed_settings settings){
    int track_midpoint_idx = 64;
    int delta = 0;
    double new_speed = 0.0;
    double slowdown_scalar = 0.0;
    double desired;

    if (running){

        track_midpoint_idx = line_stats.x;
        delta = 64-track_midpoint_idx;
        
        if (delta < 0)
            delta = delta * -1;
        
        slowdown_scalar = ((double)delta)/7.0;
        
        if (line_stats.y < CARPET_THRESHOLD){
            //desired = settings.corner_speed;
            desired = settings.corner_speed;
            slowdown_scalar = slowdown_scalar * 1.5;
        }
        else{
            desired = settings.straight_speed;
            slowdown_scalar = slowdown_scalar;
        }
        
        desired = desired - (settings.max_corner_slowdown * slowdown_scalar);
        
        #ifdef USE_UART
        //sprintf(uart_tx_buffer, "delta = %d,   scale = %g,    desired = %g\n\r", delta, slowdown_scalar, desired);
        //uart0_put(uart_tx_buffer);
        #endif 

        new_speed = DrivingPID(pid_params, desired, current_speed);


        if (delta == 0){
            TIMER_A0_PWM_DutyCycle(new_speed/100.0, LEFT_MOTOR);
            TIMER_A0_PWM_DutyCycle(new_speed/100.0, RIGHT_MOTOR);
        }
        else{
            if (track_midpoint_idx < 64){   //  making a left turn
                TIMER_A0_PWM_DutyCycle((new_speed - settings.inner_wheel_slowdown)/100.0, LEFT_MOTOR);  // Inner wheel
                TIMER_A0_PWM_DutyCycle((new_speed + settings.outer_wheel_speedup)/100.0, RIGHT_MOTOR);   // Outer wheel
            }
            else {  //  Making a right turn
                TIMER_A0_PWM_DutyCycle((new_speed + settings.outer_wheel_speedup)/100.0, LEFT_MOTOR);    // Outer wheel
                TIMER_A0_PWM_DutyCycle((new_speed - settings.inner_wheel_slowdown)/100.0, RIGHT_MOTOR); // Inner wheel
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
center_of_mass_t parseCameraData(uint16_t* raw_camera_data, uint16_t* smoothed_camera_data){
    center_of_mass_t line_stats;
    
    if (g_sendData == TRUE){
        LED1_On();

        MovingAverage(raw_camera_data, smoothed_camera_data); // smooth raw data
        FindCenterOfMass(smoothed_camera_data, &line_stats);

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
    if ((10 < camera_max) && (camera_max < CARPET_THRESHOLD)){  // if max val is too low (darkness/carpet)
        return TRUE;    // car is off track
    }
    else{
        return FALSE; // car is not off track
    }
}

void ChooseRaceSettings(speed_settings* settings){
    for (;;){
        switch (raceMode) {
            case Jog:
                LED2_Red();
                settings->straight_speed = 35.0;
                settings->corner_speed = 32.5;
                settings->inner_wheel_slowdown = 3.0;
                settings->outer_wheel_speedup = 10.0;
                settings->max_corner_slowdown = 11.0;
                break;
            case Run:
                LED2_Green();
                settings->straight_speed = 40.5;
                settings->corner_speed = 32.5;
                settings->inner_wheel_slowdown = 4.0;
                settings->outer_wheel_speedup = 10.0;
                settings->max_corner_slowdown = 13.0;
                break;
            case Sprint:
                LED2_Blue();
                settings->straight_speed = 44.5;
                settings->corner_speed = 32.5;
                settings->inner_wheel_slowdown = 5.0;
                settings->outer_wheel_speedup = 10.0;
                settings->max_corner_slowdown = 14.5;
                break;
            default:
                LED2_Off();
                break;
        }
        
        if (Switch1_Pressed()){
            raceMode++;
            if (raceMode > 2)
                raceMode = Jog;
            while(Switch1_Pressed()){}
        }
        else if (Switch2_Pressed()){
            LED2_Off();
            running = TRUE;
            while(Switch2_Pressed()){}
            break;
        }
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
    center_of_mass_t line_statistics;   // stats of camera data
    int track_loss_counter = 0;     // off track counter
    double motor_speed = 20.0;
    
    // start with sprint speed settings
    speed_settings speedSettings;

    // PID variables for performance tuning
    /*
      Started with on 4/18/22
      kp = 0.13
      ki = 0.05
      kd = 0.0
    */
    //pid_values_t steering_pid = {0.45, 0.03, 0.3};
    //pid_values_t steering_pid = {0.08, 0.0, 0.0};
    //pid_values_t steering_pid = {0.006, 0.0, 0.0};
    pid_values_t steering_pid = {0.005, 0.025, 0.045};
    pid_values_t driving_pid = {0.32, 0.06, 0.148};

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
    raceMode = Run;  //  Jog will show up as first race mode
    LED2_Off();

    ChooseRaceSettings(&speedSettings);
    
    for (;;){
        
        /* Read camera data */
        line_statistics = parseCameraData(line, smoothed_line);

        /* Adjust steering direction */
        adjustSteering(line_statistics, steering_pid);
        
        /* Adjust DC Motor speed */
        motor_speed = adjustDrivingContinuous(line_statistics, driving_pid, motor_speed, speedSettings);

        /* Check for track loss or intersection */
        if (isOffTrack(line_statistics.y))
            track_loss_counter += 1;    //  if off track, increment counter
        else
            track_loss_counter = 0;     //  if on track, reset counter to zero

        /* Carpet detection reached limit */
        if (track_loss_counter > TRACK_LOSS_LIMIT){
            running = FALSE;
        }
        else if (Switch2_Pressed()){    // Re-enable car into the previously selected mode
            track_loss_counter = 0;
            LED2_Yellow();
            while(Switch2_Pressed()){}
            LED2_Off();
            running = TRUE;
        }

        #ifdef USE_UART
            //sprintf(uart_tx_buffer, "servo: %g;  left line idx:  %d;  right line idx:  %d;\n\r", servo_position, line_statistics.left_slope_index, line_statistics.right_slope_index);
            sprintf(uart_tx_buffer, "X = %d,   Y = %d\n\r", line_statistics.x, line_statistics.y);
            uart0_put(uart_tx_buffer);
            //uart2_put(uart_tx_buffer);
        #endif

		/* Do a small delay */
		msdelay(10);
    }
}
