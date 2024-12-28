#include "user.h"
#include "uart0.h"

#include <hardware/uart.h>
#include <hardware/irq.h>

static on_byte_callback callback;

void UART0_RxInterruptHandler()
{
    while (uart_is_readable(uart0))
    {
        uint8_t byteIn = uart_getc(uart0);

        if (callback != nullptr)
        {
            callback(byteIn);
        }
    }
}

void UART0_Init()
{
    int __unused actual = uart_set_baudrate(uart0, UART0_BAUD);

    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, UART0_DATABITS, UART0_STOPBITS, UART0_PARITY);
    uart_set_fifo_enabled(uart0, false);
    
    irq_set_exclusive_handler(UART0_IRQ, UART0_RxInterruptHandler);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(uart0, true, false);

    callback = nullptr;
}

void UART0_SetByteRxCallback(on_byte_callback o_callback)
{
    callback = o_callback;
}