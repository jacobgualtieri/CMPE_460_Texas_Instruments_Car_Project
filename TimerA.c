// TimerA.c

/*  LJBeato
    2021
    TimerA functionality to drive DC motor
    and Servo Motor
 */


#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "TimerA.h"
#include "uart.h"

// Make these arrays 5 deep, since we are using indexes 1-4 for the pins
static uint32_t DEFAULT_PERIOD_A0[5] = {0,0,0,0,0};
static uint32_t DEFAULT_PERIOD_A2[5] = {0,0,0,0,0};


unsigned long CalcPeriodFromFrequency(double Hz){
	double period = 0.0;
	period = (double)SystemCoreClock/Hz;
	period = period;   // we divide by 2 because we want an interrupt for both the rising edge and the falling edge
	return (unsigned long) period;
}

//***************************PWM_Init*******************************
// PWM output on P2.4, P2.5, P2.6, P2.7
// Inputs:  period of P2.4...P2.7 is number of counts before output changes state
//          percentDutyCycle (0 -> 1.0)
//          pin number (1,2,3,4)
// Outputs: none
int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin){

	uint16_t dutyCycle;

	// Timer A0.1
	if (pin == 1){
        P2->SEL0 |= BIT4;
        P2->SEL1 &= ~BIT4;

		P2->DIR |= BIT4;
		P2->OUT &= ~BIT4;
	}
    // Timer A0.2
	else if (pin == 2){
        P2->SEL0 |= BIT5;
        P2->SEL1 &= ~BIT5;

		P2->DIR |= BIT5;
		P2->OUT &= ~BIT5;
	}
    // Timer A0.3
	else if (pin == 3){
        P2->SEL0 |= BIT6;
        P2->SEL1 &= ~BIT6;

		P2->DIR |= BIT6;
		P2->OUT &= ~BIT6;
	}
    // Timer A0.4
	else if (pin == 4){
        P2->SEL0 |= BIT7;
        P2->SEL1 &= ~BIT7;

		P2->DIR |= BIT7;
		P2->OUT &= ~BIT7;
	}
	else return -2;

	// save the period for this timer instance
	// DEFAULT_PERIOD_A0[pin] where pin is the pin number
	DEFAULT_PERIOD_A0[pin] = period;
	// TIMER_A0->CCR[0]
	TIMER_A0->CCR[0] = DEFAULT_PERIOD_A0[pin];


	// TIMER_A0->CCTL[pin]
	// 16 bit register
	// 15-14    CM          Capture Mode                        00b = no capture
	// 13-12    CCIS        Capture/compare input select        10b = GND
	// 11       SCS         Synchronize capture source          0b = asynchronous capture
	// 10       SCCI        Synchronized capture/compare input  0b
	// 9        RESERVED                                        0b
	// 8        CAP         Capture Mode                        0b = compare mode
	// 7-5      OUTMOD      Output mode                         111b = Reset/Set
	// 4        CCIE        Capture/Compare Interrupt enable    1b = interrupt enabled
	// 3        CCI         Capture/Compare input (read-only)   0b read only
	// 2        OUT         Output                              1b = output high
	// 1        COV         Capture overflow                    0b = no overflow occurred
	// 0        CCIFG       Capture/Compare interrupt flag      0b = no interrupt pending
	TIMER_A0->CCTL[pin] = 0x20F4;

	// set the duty cycle
	dutyCycle = (uint16_t) (percentDutyCycle * (double)DEFAULT_PERIOD_A0[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
    //TIMER_A0->CCR[pin]
    TIMER_A0->CCR[pin] = dutyCycle;

	/*
	 Timer CONTROL register
     TIMER_A0->CTL
	 bits 15-10 : reserved
	 bits 9-8   : clock source select   : 10b = SMCLK
	 bits 7-6   : input divider         : 00b = /1
	 bits 5-4   : mode control          : 01b = Up mode
	 bit  3     : reserved              : 0b
	 bits 2-1   : clear, enables        : 11b = cleared TAxR, enable interrupts
	 bit  0     : interrupt pending     : 0b
	*/
	TIMER_A0->CTL |= 0x0216;
	return 0;
}
//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on pin
// Inputs:  duty-cycle, pin
// Outputs: none
// percentDutyCycle is a number between 0 and 1  (ie. 0.5 = 50%)
void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin)
{
	TIMER_A0->CCR[pin] = (uint16_t) (percentDutyCycle * (double)DEFAULT_PERIOD_A0[pin]);
}

//***************************PWM_Init*******************************
// PWM output on P5.6
// Inputs:  period of P5.6 is number of counts before output changes state
//          percentDutyCycle (0 -> 1.0)//          duty cycle
//          pin number (1,2,3,4), but always 1
// Outputs: none
int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin){

    uint16_t dutyCycle;
    
    //  Timer_A2.1 -> P5.6
    //  Timer_A2.2 -> P5.7
    //  Timer_A2.3 -> P6.6
    //  Timer_A2.4 -> P6.7

	//  NOTE: Timer A2 only exposes 1 PWM pin
	//  Timer_A2.1 -> P5.6
	if (pin == 1){
        P5->SEL0 |= BIT6;
        P5->SEL1 &= ~BIT6;

        P5->DIR |= BIT6;
        P5->OUT &= ~BIT6;
	}
	else return -2;

    // NOTE: Setup similar to TimerA0
    // You will have to use the pre-scalar (clock divider) to get down to 20ms
    //  TODO: 19.3.6 TAxEX0 Register: Figure out which pre-scalar is the right one
    //  2-0     TAIDEX      Input divider expansion
    //          000b    divide by 1
    //          001b    divide by 2
    //          010b    divide by 3
    //          011b    divide by 4
    //          100b    divide by 5
    //          101b    divide by 6
    //          110b    divide by 7
    //          111b    divide by 8
    TIMER_A2->EX0 = 0x0004;

    // save the period for this timer instance
    // DEFAULT_PERIOD_A0[pin] where pin is the pin number
    DEFAULT_PERIOD_A2[pin] = period;
    // TIMER_A2->CCR[0]
    TIMER_A2->CCR[0] = DEFAULT_PERIOD_A2[pin];


    // TIMER_A2->CCTL[pin]
    // 16 bit register
    // 15-14    CM          Capture Mode                        00b = no capture
    // 13-12    CCIS        Capture/compare input select        10b = GND
    // 11       SCS         Synchronize capture source          0b = asynchronous capture
    // 10       SCCI        Synchronized capture/compare input  0b
    // 9        RESERVED                                        0b
    // 8        CAP         Capture Mode                        0b = compare mode
    // 7-5      OUTMOD      Output mode                         111b = Reset/Set
    // 4        CCIE        Capture/Compare Interrupt enable    1b = interrupt enabled
    // 3        CCI         Capture/Compare input (read-only)   0b read only
    // 2        OUT         Output                              1b = output high
    // 1        COV         Capture overflow                    0b = no overflow occurred
    // 0        CCIFG       Capture/Compare interrupt flag      0b = no interrupt pending
    TIMER_A2->CCTL[pin] = 0x20F4;

    // CCR[n] contains the dutyCycle just calculated, where n is the pin number
    dutyCycle = (uint16_t) (percentDutyCycle * (double)DEFAULT_PERIOD_A2[pin]); // set the duty cycle
    TIMER_A2->CCR[pin] = dutyCycle;

    /*
     Timer CONTROL register
     TIMER_A2->CTL
     bits 15-10 : reserved
     bits 9-8   : clock source select   : 10b = SMCLK
     bits 7-6   : input divider         : 10b = /4
     bits 5-4   : mode control          : 01b = Up mode
     bit  3     : reserved              : 0b
     bits 2-1   : clear, enables        : 11b = cleared TAxR, enable interrupts
     bit  0     : interrupt pending     : 0b
    */
    TIMER_A2->CTL = 0x0296;
	return 0;
}
//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P5.6
// Inputs:  percentDutyCycle, pin  (should always be 1 for TimerA2.1)
//
// Outputs: none
//
void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin)
{
	TIMER_A2->CCR[pin] = (uint16_t) (percentDutyCycle * (double)DEFAULT_PERIOD_A2[pin]);
}

