#include "msp.h"
#include "Timer32.h"
#include "Common.h"
#include "CortexM.h"
// Sections borrowed from Jonathan Valvano UTexas
extern uint32_t SystemCoreClock;

// NOTE: By default the MCLK is running at 3MHz
double systemTimer = 0;
void (*Timer32_1_PeriodicTask)(void);   // user function
void (*Timer32_2_PeriodicTask)(void);   // user function

static unsigned long timer1Period;
static unsigned long timer2Period;
///////////////////////////////////////////////////////
//
// Helper function
//
///////////////////////////////////////////////////////
unsigned long  CalcPeriodFromFrequency (double Hz)
{
	double period = 0.0;
	period = (double)SystemCoreClock/Hz;
	period = period;   // we divide by 2 because we want an interrupt for both the rising edge and the falling edge
	return (unsigned long) period;
}

BOOLEAN timeOut = FALSE;
// this will timeout after 1 millisecond
void MS_Timeout_Handler(void)
{
	timeOut = TRUE;
}

// ***************** Timer32_1_Init ****************
// Activate Timer32 Timer 1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/(bus clock)/div), 32 bits
//          div is clock divider for Timer32 Timer 1
//            T32DIV1   for input clock divider /1
//            T32DIV16  for input clock divider /16
//            T32DIV256 for input clock divider /256
// NOTE: The default clock is 3MHz
// Outputs: none
void Timer32_1_Init(void(*task)(void), unsigned long period, enum timer32divider div)
{
	long sr;
	timer1Period = period;
	// default MCLK is 3MHz
	// but set MCLK to 48 MHz
  sr = StartCritical();

	// unsigned long function
  Timer32_1_PeriodicTask = task;

	// timer reload value
	// TIMER32_LOAD1
  TIMER32_LOAD1 = timer1Period;

	// clear Timer32 Timer 1 interrupt
	// TIMER32_INTCLR1
  TIMER32_INTCLR1 = 1;


  // bits31-8=X...X,   reserved
  // bit7,             timer 0=disable, 1=enable
  // bit6,             0 = Timer is in free-running mode, 1=timer in periodic mode
  // bit5,             interrupt 0=disable, 1=enable
  // bit4=X,           reserved
  // bits3-2=??,       input clock divider according to parameter
  // bit1,             0=16bit counter, 1=32-bit counter
  // bit0,             1=one shot mode, 0=wrapping mode

	// TIMER32_CONTROL1, enable, periodic, 32 bit counter
  TIMER32_CONTROL1 |= BIT7;		// Enable timer 1
  TIMER32_CONTROL1 |= BIT6;		// Periodic mode
  TIMER32_CONTROL1 |= BIT5;		// Enable interupts
  TIMER32_CONTROL1 &= ~div;
  TIMER32_CONTROL1 |= div;
  TIMER32_CONTROL1 |= BIT1;		// 32-bit mode
  TIMER32_CONTROL1 |= BIT0;   // One-shot mode

	// interrupts enabled in the main program after all devices initialized
	// NVIC_IPR6
  NVIC_IPR6 = (NVIC_IPR6&0xFFFF00FF)|0x00004000; // priority 2

	// enable interrupt 25 in NVIC, NVIC_ISER0
	// NVIC_ISER0
  NVIC_ISER0 = 1 << 25;

  EndCritical(sr);
}


void Timer32_1_Start(void){
	TIMER32_CONTROL1 |= BIT7;		// Enable timer 1
}


void Timer32_1_Stop(void){
	TIMER32_CONTROL1 &= ~BIT7;	// Disable timer 1
}


void T32_INT1_IRQHandler(void)
{
	// acknowledge Timer32 Timer 1 interrupt
	// TIMER32_INTCLR1
  TIMER32_INTCLR1 = 1;

	// execute user task
  (*Timer32_1_PeriodicTask)();

	// timer reload value to start the timer again
	// TIMER32_LOAD1
  TIMER32_LOAD1 = TIMER32_BGLOAD1;
}

// ***************** Timer32_2_Init ****************
// Activate Timer32 Timer 2 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/(bus clock)/div), 32 bits
//          div is clock divider for Timer32 Timer 1
//            T32DIV1   for input clock divider /1
//            T32DIV16  for input clock divider /16
//            T32DIV256 for input clock divider /256
// NOTE: The default clock is 3MHz
// Outputs: none
void Timer32_2_Init(void(*task)(void), unsigned long period, enum timer32divider div)
{
	long sr;
	timer2Period = period;

	// default MCLK is 3MHz
	// but set MCLK to 48 MHz
  sr = StartCritical();

	// unsigned long function
	// assigns the ISR
  Timer32_2_PeriodicTask = task;

	// timer reload value
	// TIMER32_LOAD2
  TIMER32_LOAD2 = timer2Period;

	// clear Timer32 Timer 2 interrupt
	// TIMER32_INTCLR2
  TIMER32_INTCLR2 = 1;


  // bits31-8=X...X,   reserved
  // bit7,             timer 0=disable, 1=enable
  // bit6,             0 = Timer is in free-running mode, 1=timer in periodic mode
  // bit5,             interrupt 0=disable, 1=enable
  // bit4=X,           reserved
  // bits3-2=??,       input clock divider according to parameter
  // bit1,             0=16bit counter, 1=32-bit counter
  // bit0,             1=one shot mode, 0=wrapping mode

  //TIMER32_CONTROL2
  TIMER32_CONTROL2 |= BIT7;  // Enable timer 2
  TIMER32_CONTROL2 |= BIT6;   // Periodic mode
  TIMER32_CONTROL2 |= BIT5;   // Enable interrupts
  TIMER32_CONTROL2 &= ~div;
  TIMER32_CONTROL2 |= div;
  TIMER32_CONTROL2 |= BIT1;   // 32-bit mode
  TIMER32_CONTROL2 |= BIT0;   // One-shot mode

	// interrupts enabled in the main program after all devices initialized
  NVIC_IPR6 = (NVIC_IPR6&0xFFFF00FF)|0x00004000; // priority 2

	// enable interrupt 26 in NVIC, NVIC_ISER0
	// NVIC_ISER0
  NVIC_ISER0 = 1 << 26;

  EndCritical(sr);
}



void T32_INT2_IRQHandler(void)
{
	// acknowledge Timer32 Timer 1 interrupt
	// TIMER32_INTCLR2
  TIMER32_INTCLR2 = 1;

	// execute user task
  (*Timer32_2_PeriodicTask)();

	// timer reload value
	// TIMER32_LOAD2
  TIMER32_LOAD2 = TIMER32_BGLOAD2;

}


void Timer32_2_Start(void){
	TIMER32_CONTROL2 |= BIT7;		// Enable timer 2
}


void Timer32_2_Stop(void){
	TIMER32_CONTROL2 &= ~BIT7;	// Disable timer 2
}
