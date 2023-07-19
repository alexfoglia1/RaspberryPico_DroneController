#include "timer.h"
#include "attitude.h"
#include "servo.h"
#include "pulse.h"
#include "joystick.h"
#include "motors.h"
#include <stdio.h>


static repeating_timer_t timer;
bool TIMER_Init(int frequency_hz)
{
    timer = {};

    int period_us = SECONDS_TO_US / frequency_hz;
    return add_repeating_timer_us(-period_us, __TIMER_ISR__, NULL, &timer);
}


void TIMER_Loop()
{
    ATTITUDE_Handler();
    JOYSTICK_Handler();
    MOTORS_Handler();
}


bool __TIMER_ISR__(repeating_timer_t* timer)
{
    TIMER_Loop();

    return true;
}

