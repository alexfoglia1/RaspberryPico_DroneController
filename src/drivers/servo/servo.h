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
    uint32_t currentSignal();
private:
    int _gpio;
    uint32_t _slice_num;
    uint32_t _chan;
    uint32_t _period_us;

    uint32_t pwm_set_freq_duty(uint32_t f, float dc);

};

#endif