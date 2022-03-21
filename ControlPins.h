#ifndef _CONTROL_PINS_
#define _CONTROL_PINS_
#include "Common.h"

// SI Pin will be P5.5 A0
// CLK Pin will be P5.4 A1

#define SI BIT5
#define CLK BIT4
void ControlPin_SI_Init(void);
void ControlPin_CLK_Init(void);
void CLK_Handler(void);
void SI_Handler(void);

#endif
