#include "msp.h"
#include "Common.h"
#include "switches.h"

typedef unsigned char BOOLEAN;
#define FALSE 0
#define TRUE  !FALSE
typedef unsigned char BYTE;

void Switch1_Init(void){
	// Switch 1 P1.1
	// configure SW1 to be interrupted on the falling edge

	// enable input with pullup resistor
	P1->REN |= BIT1;
	P1->OUT |= BIT1;

	// dir=0
	// configure as input
	P1->DIR &= ~BIT1;

	// configure PortPin for Switch 1 as port I/O
	// SEL0=0 SEL1=0
	P1->SEL0 &= ~BIT1;
	P1->SEL1 &= ~BIT1;

}

void Switch2_Init(void){
	// Switch 2 P1.4
	// configure SW2 to be interrupted on the falling edge

	// enable input with pullup resistor
	P1->REN |= BIT4;
	P1->OUT |= BIT4;

	// dir=0
	// configure as input
	P1->DIR &= ~BIT4;


	// configure PortPin for Switch 2 as port I/O
	// SEL0=0 SEL1=0
	P1->SEL0 &= ~BIT4;
	P1->SEL1 &= ~BIT4;

}

//------------Switch_Input------------
// Read and return the status of Switch1
// Input: none
// return: TRUE if pressed
//         FALSE if not pressed
BOOLEAN Switch1_Pressed(void){
	BOOLEAN retVal = FALSE;
	// check if pressed... '0' when pressed, '1' when not pressed
	// switch1 P1.1
	if( (P1->IN & BIT1) == 0 ){ retVal = TRUE; }
	return (retVal);	// return TRUE(pressed) or FALSE(not pressed)
}

//------------Switch_Input------------
// Read and return the status of Switch2
// Input: none
// return: TRUE if pressed
//         FALSE if not pressed
BOOLEAN Switch2_Pressed(void){
	BOOLEAN retVal = FALSE;
	// check if pressed... '0' when pressed, '1' when not pressed
	// switch2 P1.4
	if( (P1->IN & BIT4) == 0 ){ retVal = TRUE; }
	return (retVal);	// return TRUE(pressed) or FALSE(not pressed)
}
