#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define PWM_FREQUENCY_HZ 50

class Servo
{
public:
    Servo(int gpio);
    void attach();
    void writeMicroseconds(uint32_t period_us);
private:
    int _gpio;
    uint32_t _slice_num;
    uint32_t _chan;

    uint32_t pwm_set_freq_duty(uint32_t slice_num, uint32_t chan,uint32_t f, int d);

};

#endif