#include <xc.h>
#include "pwm.h"

void PWM_Init(void) {   
    RA0PPS = 0x0D;   //RA0->PWM5:PWM5;  
    RA1PPS = 0x0E;   //RA1->PWM6:PWM6;   
    RA3PPS = 0x0F;   //RA3->PWM7:PWM7;
    RA4PPS = 0x10;   //RA4->PWM8:PWM8;  
    
    PWM5CON = 0x80;    
    CCPTMRS1bits.P5TSEL = 1;
    PWM6CON = 0x80;    
    CCPTMRS1bits.P6TSEL = 1;
    PWM7CON = 0x80;     
    CCPTMRS1bits.P7TSEL = 1;
    PWM8CON = 0x80;    
    CCPTMRS1bits.P8TSEL = 1;
}

void PWM_Set(uint8_t byte1, uint8_t byte2) {
    uint8_t select = byte1 & 0x03;
    uint8_t duty1 = byte1 & 0xC0;
    uint8_t duty2 = byte2;
    
    if (select == 0x00) {
        //8 MSB
        PWM5DCH = duty2;
        //2 LSB
        PWM5DCL = duty1;
    } else if (select == 0x40) {
        PWM6DCH = duty2;
        PWM6DCL = duty1;
    } else if (select == 0x80) {
        PWM7DCH = duty2;
        PWM7DCL = duty1;
    } else if (select == 0xC0) {
        PWM8DCH = duty2;
        PWM8DCL = duty1;
    } else {
        // Should never happen
    }
 }
