#ifndef UART_H
#define	UART_H

#include <xc.h> 

void UART_Init(void);
void UART_Read_Line(uint8_t* string);
void UART_Write(uint8_t byte);
void UART_Write_Line(uint8_t* string);
void UART_Read_Buffer();
void UART_RX_ISR(void);



#endif

