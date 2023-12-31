#include "user.h"
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>


const uint8_t MAJOR_V = 1;
const uint8_t MINOR_V = 0;
const uint8_t STAGE_V = 27;
const REL_TYPE_TAG REL_TYPE = REL_TYPE_TAG::BETA;


void InitBoard()
{
    stdio_usb_init();

    gpio_init(PROBE_GPIO_CPU0);
    gpio_set_dir(PROBE_GPIO_CPU0, GPIO_OUT);

    gpio_init(PROBE_GPIO_CPU1);
    gpio_set_dir(PROBE_GPIO_CPU1, GPIO_OUT);    

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    irq_set_priority(IO_IRQ_BANK0, 0x40);
}