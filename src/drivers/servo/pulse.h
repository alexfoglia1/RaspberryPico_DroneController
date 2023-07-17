#ifndef PULSE_H
#define PULSE_h

#include <hardware/gpio.h>
#include <stdint.h>
#include <vector>


void gpio_irq_entry_point(uint gpio, uint32_t events);

class Pulse
{
public:
    Pulse(int gpio);
    void attach();
    uint64_t pulseIn();

    void rise(uint gpio, uint64_t time_us);
    void fall(uint gpio, uint64_t time_us);
    
private:
    int _gpio;
    uint64_t _t_rise_us;
    uint64_t _duration_us;
};

extern std::vector<Pulse*> pulse_in_vector;
#endif