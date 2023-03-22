#include <xc.h>
#include "timer.h"
#include <stdbool.h>

extern bool encoder_flag;
static volatile uint16_t timer_count = 0;
void Timer_Init(void) {
    T2CLKCON = 0x01;
    T2PR = 0xF9;
    T2TMR = 0x00;
    PIR4bits.TMR2IF = 0;
    T2CON = 0xA0;
}

void Timer_ISR(void) {
    PIR4bits.TMR2IF = 0;
    if (timer_count >= 1000) {
        timer_count = 0;
        encoder_flag = true;
    }
    timer_count++;
}
