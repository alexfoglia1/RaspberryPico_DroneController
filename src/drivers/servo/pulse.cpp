#include "pulse.h"
#include <stdio.h>
#include <pico/stdlib.h>

std::vector<Pulse*> pulse_in_vector;

void gpio_irq_entry_point(uint gpio, uint32_t events)
{
    uint64_t time_us = time_us_64();

    for (Pulse* pulse : pulse_in_vector)
    {
        if (events == GPIO_IRQ_EDGE_RISE)
        {
            pulse->rise(gpio, time_us);
        }
        else
        {
            pulse->fall(gpio, time_us);
        }
        
    }
}

Pulse::Pulse(int gpio)
{
    _gpio = gpio;
    _t_rise_us = 0;
    _duration_us = 0;
}


void Pulse::attach()
{
    pulse_in_vector.push_back(this);
    
    gpio_set_dir(_gpio, GPIO_IN);
    gpio_set_irq_enabled_with_callback(_gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_entry_point);
}


uint64_t Pulse::pulseIn()
{
    return _duration_us;
}


uint64_t Pulse::lastRise_us()
{
    return _t_rise_us;
}


void Pulse::rise(uint gpio, uint64_t time_us)
{
    if ((int)gpio == _gpio)
    {
        _t_rise_us = time_us;
    }
}


void Pulse::fall(uint gpio, uint64_t time_us)
{
    if ((int)gpio == _gpio)
    {
        _duration_us = time_us - _t_rise_us;
    }
}