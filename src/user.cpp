#include "user.h"
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>


void InitBoard()
{
    stdio_init_all();

    i2c_init(i2c_default, I2C_FREQUENCY_HZ);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    gpio_init(PROBE);
    gpio_set_dir(PROBE, GPIO_OUT);
}