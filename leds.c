/* 
Title:  	Lab 1: Intro  to  MSP432  GPIO  and  Keil  MicroVision
Purpose:	Become familiar with interfacing with the MSP432 board. Use	GPIO
					to control on board LEDs using on board switches as inputs.
Name:   	Zeb Hollinger	
Date: 		01/19/22

This file prepared by LJBeato
01/11/2021
  
*/
#include "msp.h" 
#include "Common.h"

#include "leds.h"

typedef unsigned char BOOLEAN;
#define FALSE 0
#define TRUE  !FALSE
typedef unsigned char BYTE;

void LED1_Init(void) {
	// LED1 P1.0
	// configure PortPin for LED1 as port I/O 
	P1->SEL0 &= ~BIT0;
	P1->SEL1 &= ~BIT0;
	
	// make built-in LED1 have high drive strength
	P1->DS |= BIT0; //set high drive strength
	
	// set BIT 0 as output
	P1->DIR |= BIT0; // set DIR to '1'

	// turn off LED
	P1->OUT &= ~BIT0; // set P1.0 to '0'
	
}

void LED1_On(void) {
	//turn on LED1
	P1->OUT |= BIT0; // set P1.0 to '1'
}

void LED1_Off(void) {
	//turn off LED1
	P1->OUT &= ~BIT0; // set P1.0 to '0'
}

void LED2_Init(void) {
	// LED2 RED P2.0
	// LED2 Green P2.1
	// LED2 Blue P2.2
	// configure PortPin for LED2 as port I/O 
	P2->SEL0 &= ~BIT0; // LED2 - Red
	P2->SEL1 &= ~BIT0;
	P2->DIR |= BIT0; // set DIR to '1' for output

	P2->SEL0 &= ~BIT1; // LED2 - Green
	P2->SEL1 &= ~BIT1;
	P2->DIR |= BIT1; // set DIR to '1' for output

	P2->SEL0 &= ~BIT2; // LED2 - Blue
	P2->SEL1 &= ~BIT2;
	P2->DIR |= BIT2; // set DIR to '1' for output
	
	// make built-in LED2 LEDs high drive strength
	P2->DS |= BIT0; // LED2 - Red
	P2->DS |= BIT1; // LED2 - Green
	P2->DS |= BIT2; // LED2 - Blue

	// turn off LED
	P2->OUT &= ~BIT0; // LED2 - Red
	P2->OUT &= ~BIT1; // LED2 - Green
	P2->OUT &= ~BIT2; // LED2 - Blue
 
}

void LED2_Off(void){
	// turn off all 3 colors in LED2
	P2->OUT &= ~BIT0; // turn red off
	P2->OUT &= ~BIT1; // turn green off
	P2->OUT &= ~BIT2; // turn blue off
}

void LED2_Red(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on red color LED2
	P2->OUT |= BIT0;
}

void LED2_Green(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on Green color LED2
	P2->OUT |= BIT1;
}

void LED2_Blue(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on Blue color LED2
	P2->OUT |= BIT2;
}

void LED2_Cyan(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on Cyan color LED2
	//C -> Green + Blue
	P2->OUT |= BIT1; // turn green on
	P2->OUT |= BIT2; // turn blue on 
}

void LED2_Magenta(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on Magenta color LED2
	//M -> Red + Blue
	P2->OUT |= BIT0; // turn red on
	P2->OUT |= BIT2; // turn blue on 
	
}

void LED2_Yellow(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on Yellow color LED2
	//Y -> Red + Green
	P2->OUT |= BIT1; // turn green on
	P2->OUT |= BIT0; // turn red on
	
}

void LED2_White(void) {
	LED2_Off(); // clear all other colors that are on
	//turn on White color LED2
	//White -> Red + Green + Blue
	P2->OUT |= BIT0; // turn red on
	P2->OUT |= BIT1; // turn green on
	P2->OUT |= BIT2; // turn blue on 
}

// BOOLEAN LED1_State(void){
// 	BOOLEAN retVal = FALSE;
	
// 	if ((P1->OUT & BIT1) != BIT1){
// 		retVal = TRUE;
// 	}
// 	else retVal = FALSE;

// 	return retVal;
// }
