#include "user.h"
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>


void InitBoard()
{
    stdio_usb_init();

    gpio_init(PROBE_GPIO);
    gpio_set_dir(PROBE_GPIO, GPIO_OUT);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    irq_set_priority(IO_IRQ_BANK0, 0x40);
}