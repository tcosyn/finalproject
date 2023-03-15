#include <xc.h>
#include "uart.h"
#include <string.h>
#include <stdbool.h>

extern bool command_flag;
static volatile uint8_t UART_RX_read_ptr;
static volatile uint8_t UART_RX_buffer[32];

void UART_Init(void) {
    PIE3bits.U1RXIE = 0;
    RC6PPS = 0x13;   //RC6->UART1:TX1;    
    U1RXPPS = 0x17;   //RC7->UART1:RX1;
    U1CON0 = 0xB0;
    U1CON1 = 0x80;
    U1BRGL = 0x67;
    
    UART_RX_read_ptr = 0;
    command_flag = false;
    
    // enable receive interrupt
    PIE3bits.U1RXIE = 1;
    // enable 
    U1CON0bits.RXEN = 1; 
    U1CON0bits.TXEN = 1; 
}

//transfer contents of buffer up to newline character to the pointer argument
void UART_Read_Line(uint8_t* string) {
    for (uint8_t i = 0; i < 32; ++i) {
        string[i] = UART_RX_buffer[i];
    }
    UART_RX_read_ptr = 0;
}

//write byte to UART
void UART_Write(uint8_t byte) {
    while (!PIR3bits.U1TXIF) {}
    U1TXB = byte;
}

// write string to UART up to newline
void UART_Write_Line(uint8_t* string) {
    uint8_t* ptr = string;
    while (*ptr != '\n') {
        UART_Write(*ptr);
        ptr++;
    }
}

//read from UART register and store in buffer, if newline, then set flag indicating complete command
void UART_RX_ISR(void) {
    UART_RX_buffer[UART_RX_read_ptr] = U1RXB;
    if (UART_RX_buffer[UART_RX_read_ptr] == '\n')
        command_flag = true;
    UART_RX_read_ptr++;
}
