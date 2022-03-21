#include <stdint.h>
#include "msp.h"
#include "SysTickTimer.h"
#define CLOCK_SPEED 12000000
extern uint32_t SystemCoreClock;

void (*SysTickPeriodicTask)(void);   // user function
long g_SysTickTimerCounter = 0;

//
// Enable SysTick Timer
//
void EnableSysTickTimer(void){
	// enable SysTick with core clock and interrupts
	//SYSTICK_STCSR
	// 31-17    RESERVED
	// 16       COUNTFLAG
	// 15-3     RESERVED
	// 2        CLKSOURCE
	// 1        TICKINT
	// 0        ENABLE 0=disabled, 1=enabled
	SYSTICK_STCSR |= BIT0;  // set ENABLE bit to 1
}

//
// Disable SysTick Timer
//
void DisableSysTickTimer(void){
	// disable SysTick with core clock and interrupts
	// SYSTICK_STCSR
	// 31-17    RESERVED
	// 16       COUNTFLAG
	// 15-3     RESERVED
	// 2        CLKSOURCE
	// 1        TICKINT
	// 0        ENABLE 0=disabled, 1=enabled
	SYSTICK_STCSR &= ~BIT0;  // set ENABLE bit to 0
}

// numIntsPerSec is equal to the number is the
// number of times per second the SysTickHandler is going to be called.
// If you are making a clock function, you will have to double the number, so
// you can flip the bit high and then low on successive clocks.
// remember if you want a 100 Hz clock you will need 200 Interrupts per second
void SysTickTimer_Init(void(*task)(void), unsigned long period){
	long sr = 0;
	sr = StartCritical();

	// load the interrupt service routine
	SysTickPeriodicTask = task;

	// Control and Status Register
	// 1) disable SysTick during setup
	// SYSTICK_STCSR
	DisableSysTickTimer();

	// NOTE: The STRVR - RELOAD VALUE REGISTER is a 24 bit value
	// 2) reload value sets period
	// SYSTICK_STRVR
	SYSTICK_STRVR = period-1;

	// Current Value Register
	// 3) any write to current clears it
	// SYSTICK_STCVR
	// 31-24    RESERVED    R/W
	// 23-0     CURRENT     R/W     No read-modify-write protection is provided!
	SYSTICK_STCVR |= 0x0000BEEF; 

	// priority 2
	// SCB_SHPR3
	// 31-24 PRI_15 R/W 0h Priority of system handler 15.
	// 23-16 PRI_14 R/W 0h Priority of system handler 14.
	// 15-8 PRI_13 R/W 0h Priority of system handler 13.
	// 7-0 PRI_12 R/W 0h Priority of system handler 12
	SCB_SHPR3 = (SCB_SHPR3&0x00FFFFFF)|0x40000000;


	// Control and Status Register
	// 31-17 RESERVED
	// 2 CLKSOURCE R 1h Clock source.
	//	0b = Not applicable
	//	1b = Core clock
	//	1 TICKINT R/W 0h
	//	0 ENABLE R/W 0h Enable SysTick counter
	//	0b (R/W) = Counter disabled
	// --------------------------------------------------
	// 4) enable SysTick with core clock and interrupts
	// SYSTICK_STCSR
	// TODO: Are we only supposed to be setting the CLKSOURCE to be a core clk here??
	SYSTICK_STCSR |= ENABLE_SYSTICK_CLOCK;    // set SysTick -> CLKSOURCE to be a Core CLK
	// same as SYSTICK_STCSR |= BIT2;

	EndCritical(sr);
	// the client will need to Enable Interrupts
}

void SysTick_Handler (void){
	g_SysTickTimerCounter++; // simple counter
	(*SysTickPeriodicTask)();
}
