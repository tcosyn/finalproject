/*
 * File:   main.c
 * Author: doinn
 *
 * Created on February 13, 2023, 10:53 AM
 */

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = OFF     // Multi-vector enable bit (Interrupt contoller does not use vector table to prioritze interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write-protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (HV on MCLR/VPP must be used for programming)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#include "stdbool.h"
#include "timer.h"
#include "uart.h"
#include <stdbool.h>


uint8_t current_command[32];
uint8_t left_count, right_count, prev_left_count, prev_right_count;
bool command_flag, encoder_flag, init;

void System_Init(void) {
    // Initialize oscillator
    OSCCON1 = 0x60;
    OSCCON3 = 0x00;
    OSCEN = 0x00;
    OSCFRQ = 0x02;
    OSCTUNE = 0x00;
    
    // Initialize pins
    TRISA = 0xE4;
    TRISB = 0xFF;
    TRISC = 0xBF;
    ANSELC = 0x7F;
    ANSELB = 0xFF;
    ANSELA = 0xDB; 

    INLVLA = 0xFF;
    INLVLB = 0xFF;
    INLVLC = 0xFF;
    INLVLE = 0x08;
    
    RA4PPS = 0x10;   //RA4->PWM8:PWM8;      
    RA3PPS = 0x0F;   //RA3->PWM7:PWM7;    
    RA1PPS = 0x0E;   //RA1->PWM6:PWM6;      
    RA0PPS = 0x0D;   //RA0->PWM5:PWM5
    
    //Initialize PWM
    PWM5CON = 0x80;   
    PWM5DCH = 0x00;   
    PWM5DCL = 0x00;   
    CCPTMRS1bits.P5TSEL = 1;
    
    PWM6CON = 0x80;   
    PWM6DCH = 0x00;   
    PWM6DCL = 0x00;   
    CCPTMRS1bits.P6TSEL = 1;
    
    PWM7CON = 0x80;   
    PWM7DCH = 0x00;   
    PWM7DCL = 0x00;   
    CCPTMRS1bits.P7TSEL = 1;
    
    PWM8CON = 0x80;   
    PWM8DCH = 0x00;   
    PWM8DCL = 0x00;   
    CCPTMRS1bits.P8TSEL = 1;
    
    INT0PPS = 0x02;   //RA2->EXT_INT:INT0;        
    INT1PPS = 0x05;   //RA5->EXT_INT:INT1;  
    INTCON0bits.INT0EDG = 1;
    INTCON0bits.INT1EDG = 1;
    
    //Disable vector table
    INTCON0bits.IPEN = 0;
     //Enable global interrupts
    INTCON0bits.GIE = 1;
    
    //set flags
    command_flag = false;
    encoder_flag = false;
    init = false;
}

//
void set_PWM(uint8_t select, uint16_t duty)
 {
    if (select == 5) {
        PWM5DCH = (duty & 0x03FC) >> 2;
        PWM5DCL = (duty & 0x0003) << 6;
    } else if (select == 6) {
        PWM6DCH = (duty & 0x03FC) >> 2;
        PWM6DCL = (duty & 0x0003)<<6;
    } else if (select == 7) {
        PWM7DCH = (duty & 0x03FC) >> 2;
        PWM7DCL = (duty & 0x0003) << 6;
    } else if (select == 8) {
        PWM8DCH = (duty & 0x03FC) > >2;
        PWM8DCL = (duty & 0x0003) << 6;
    } else {
        // Should never happen
    }
 }

//interrupt handler
void __interrupt() INTERRUPT_Handler (void) {
    //disable global interrupts until handled
    INTCON0bits.GIE = 0;
    if(PIE3bits.U1RXIE == 1 && PIR3bits.U1RXIF == 1) {
        UART_RX_ISR();
    }
    else if(PIE1bits.INT0IE == 1 && PIR1bits.INT0IF == 1)
    {
        left_count++;
        PIR1bits.INT0IF = 1;
    }
    else if(PIE5bits.INT1IE == 1 && PIR5bits.INT1IF == 1)
    {
        right_count++;
        PIR5bits.INT1IF = 1;
    }
    else if(PIE4bits.TMR2IE == 1 && PIR4bits.TMR2IF == 1)
    {
        Timer_ISR();
        PIR4bits.TMR2IF = 0;
    }
    else {
        // Should never happen
    }
    INTCON0bits.GIE = 1;
}

//parse command and set motor pwm if valid
void interpret_command(uint8_t* command) {
    uint8_t* error_msg = "COMMAND ERROR\n";
    uint8_t length = 0;
    uint8_t* ptr = command;
    /* Check length of message for incorrect syntax
    while (*ptr != '\n') {
        length++;
        ptr++;
    }
    if (length != 4) {
        UART_Write_Line(bad_msg);
        return;
    }
    
    ptr = command;*/
    
    //read two MSB from byte to determine left/right motor and direction
    if ((*ptr & 0xC0) == 0x00) {
        uint8_t* msg = "LF\n";
        UART_Write_Line(msg);
    } else if ((*ptr & 0xC0) == 0x40) {
        uint8_t* msg = "LB\n";
        UART_Write_Line(msg);
    } else if ((*ptr & 0xC0) == 0x80) {
        uint8_t* msg = "RF\n";
        UART_Write_Line(msg);
    } else if ((*ptr & 0xC0) == 0xC0) {
        uint8_t* msg = "RB\n";
        UART_Write_Line(msg);
    } else {
        UART_Write_Line(error_msg);
    }
}

void main(void) {
    System_Init();
    Timer_Init();
    UART_Init();

    uint8_t* command[32];
    uint8_t* ptr;
    
    //Get initialized signal from PI
    while (!init) {
        if (command_flag) {
            UART_Read_Line(command);
            ptr = command;
            UART_Write_Line(ptr);
            UART_Write('\n');
            if (*ptr == 'I' && *(ptr+1) == 'n' && *(ptr+2) == 'i' && *(ptr+3) == 't') {
                init = true;
                uint8_t* good_msg = "UART Initialized\n";
                UART_Write_Line(good_msg);
           }
            command_flag = false;
        }
    }
    
    //Enable timer interrupt to start reading from encoder
    PIE4bits.TMR2IE = 1;
    //Main Loop
    while (1) {
        //available command
        if (command_flag) {
            UART_Read_Line(command);
            ptr = command;
            UART_Write_Line(ptr);
            UART_Write('\n');
            interpret_command(ptr);
            command_flag = false;
        }
        //read encoder every second NEED IMPLEMENT)
        if (encoder_flag) {
            UART_Write('f');
            UART_Write('\n');
            encoder_flag = false;
        }
    }
}
