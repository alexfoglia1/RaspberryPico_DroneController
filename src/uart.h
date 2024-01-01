#ifndef UART_H
#define UART_H

#define UART_RX_BUFLEN 8192

#include <stdint.h>

void UART_Init();
void UART_RxInterruptHandler();
bool UART_IsReadable();
uint8_t UART_ReadByte();


#endif