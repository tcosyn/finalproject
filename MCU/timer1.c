#include <xc.h>
#include "timer1.h"

void Timer1_Init(void) {
    T1CLK = 0x01;
    TMR1H = 0xFC;
    TMR1L = 0x18;
    PIR4bits.TMR1IF = 0;
    T1CON = 0x01;
}