#ifndef UART0_H
#define UART0_H

#include <stdint.h>

#define UART0_BAUD 38400
#define UART0_DATABITS 8
#define UART0_STOPBITS 1
#define UART0_PARITY UART_PARITY_NONE

typedef void (*on_byte_callback)(uint8_t byteIn);

void UART0_Init();
void UART0_SetByteRxCallback(on_byte_callback callback);

#endif //UART0_H