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
#include "timer1.h"
#include "uart.h"
#include "ccp1.h"
#include "ccp2.h"
#include "pwm.h"
#include <stdbool.h>

//global variables
uint8_t current_command[32];
uint8_t left_freq_1, left_freq_2, right_freq_1, right_freq_2;
bool command_flag, encoder_flag, init;

//interrupt handler
void __interrupt() INTERRUPT_Handler (void) {
    if(PIE3bits.U1RXIE && PIR3bits.U1RXIF) {
        UART_RX_ISR();
    } else if (PIE4bits.CCP1IE && PIR4bits.CCP1IF) {
        CCP1_ISR();
    } else if (PIE7bits.CCP2IE && PIR7bits.CCP2IF) {
        CCP2_ISR();
    } else if (PIE4bits.TMR2IE && PIR4bits.TMR2IF) {
        Timer_ISR();
    } else {
        // Should never happen
    }
}

void System_Init(void) {
    // Initialize oscillator
    OSCCON1 = 0x60;
    OSCCON3 = 0x00;
    OSCEN = 0x00;
    OSCFRQ = 0x02;
    OSCTUNE = 0x00;
    
    // Initialize pins

    //Configure digital/analog
    TRISA = 0xE4;
    TRISB = 0xFF;
    TRISC = 0xBF;
    
    //Configure input/output
    ANSELC = 0x7C;
    ANSELB = 0xFF;
    ANSELA = 0xFF;
    
    //Disable vector table
    INTCON0bits.IPEN = 0;
     //Enable global interrupts
    INTCON0bits.GIE = 1;
    
    //Initialize flags
    command_flag = false;
    encoder_flag = false;
    init = false;
}

void main(void) {
    System_Init();
    Timer_Init();
    Timer1_Init();
    UART_Init();
    CCP1_Init();
    CCP2_Init();
    PWM_Init();
    
    //Command input buffer
    uint8_t* command[32];
    //Pointer to keep track of location
    uint8_t* ptr;
    
    //Get initialized signal from PI before proceeding
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
    
    //Enable CCP1 interrupt to calculate frequency
    PIE4bits.CCP1IE = 1;
    //Enable CCP2 interrupt to calculate frequency
    PIE7bits.CCP2IE = 1;
    //Enable timer interrupt to start sending frequency every second
    PIE4bits.TMR2IE = 1;
    
    //Main Loop
    while (1) {
        //flag for incoming command
        if (command_flag) {
            UART_Read_Line(command);
            ptr = command;
            PWM_Set(*ptr, *(ptr + 1));
            command_flag = false;
        }
        
        //flag to print encoder every second
        if (encoder_flag) {
            UART_Write(left_freq_1);
            UART_Write(left_freq_2);
            UART_Write(right_freq_1);
            UART_Write(right_freq_2);
            encoder_flag = false;
        }
    }
}
