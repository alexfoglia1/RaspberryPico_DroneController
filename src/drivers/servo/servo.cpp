#include "servo.h"
#include <pico/stdlib.h>
#include <hardware/pwm.h>


Servo::Servo(int gpio)
{
    _gpio = gpio;
    _slice_num = pwm_gpio_to_slice_num(_gpio);
    _chan = pwm_gpio_to_channel(_gpio);
    _period_us = 0;
}


void Servo::attach()
{
    gpio_set_function(_gpio, GPIO_FUNC_PWM);
    pwm_set_freq_duty(PWM_FREQUENCY_HZ, 0);
    pwm_set_enabled(_slice_num, true);
}


void Servo::writeMicroseconds(uint32_t period_us)
{
    uint32_t pwm_period_us = 1000000 / PWM_FREQUENCY_HZ;
    period_us = (period_us > pwm_period_us) ? pwm_period_us : (uint32_t)(period_us & 0xFFFFFFFF);

    float duty_cycle = ((float) period_us) / ((float)(pwm_period_us));
    pwm_set_freq_duty(PWM_FREQUENCY_HZ, duty_cycle);

    _period_us = period_us;
}


uint32_t Servo::currentSignal()
{
    return _period_us;
}


uint32_t Servo::pwm_set_freq_duty(uint32_t f, float dc)
{
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + 
                           (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0)
    {
        divider16 = 16;
    }

    uint32_t wrap = clock * 16 / divider16 / f - 1;
    pwm_set_clkdiv_int_frac(_slice_num, divider16/16, divider16 & 0xF);
    pwm_set_wrap(_slice_num, wrap);
    pwm_set_chan_level(_slice_num, _chan, wrap * dc);

    return wrap;
}