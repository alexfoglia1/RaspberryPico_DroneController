#include "uart.h"
#include "user.h"
#include "maint.h"

#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/time.h>


void UART_Init()
{
    uart_init(uart0, 2400);
    
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int __unused actual = uart_set_baudrate(uart0, BAUD_RATE);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(uart0, false);

    irq_set_exclusive_handler(UART0_IRQ , UART_RxInterruptHandler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
}


void UART_RxInterruptHandler()
{
    while (uart_is_readable(uart0))
    {
        uint8_t byteIn = uart_getc(uart0);
        MAINT_OnByteReceived(byteIn, MaintClient::UART);
    }
}