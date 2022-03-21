#include "Common.h"
#include "oled.h"
#include "msp.h"
#include "uart.h"
#include "leds.h"
#include "Timer32.h"
#include "CortexM.h"
#include "ADC14.h"
#include "ControlPins.h"

// line stores the current array of camera data
uint16_t line[128];
BOOLEAN g_sendData;

int main(void){
    for(;;){
    }
}
