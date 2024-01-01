#include "uart.h"
#include "user.h"

#include <stdint.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/time.h>
#include <string.h>

static uint8_t rx_buf[UART_RX_BUFLEN];
static uint8_t rx_idx;
static bool rx_interrupt_pending;

static void append(uint8_t byte)
{
    if (rx_idx < UART_RX_BUFLEN)
    {
        rx_buf[rx_idx] = byte;
        rx_idx += 1;
    }
}


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

    rx_idx = 0;
    memset(&rx_buf, 0, UART_RX_BUFLEN);

    rx_interrupt_pending = false;
}


void UART_RxInterruptHandler()
{
    rx_interrupt_pending = true;
    while (uart_is_readable(uart0))
    {
        uint8_t byteIn = uart_getc(uart0);
        append(byteIn);
    }
    rx_interrupt_pending = false;
}


bool UART_IsReadable()
{
    return (rx_idx > 0);
}


uint8_t UART_ReadByte()
{
    while (rx_interrupt_pending)
    {
        sleep_us(1);
    }

    int byte = rx_buf[0];

    if (rx_idx)
    {
        memcpy(&rx_buf[0], &rx_buf[1], rx_idx - 1);
        rx_buf[rx_idx - 1] = -1;
        rx_idx -= 1;
    }

    return byte;
}