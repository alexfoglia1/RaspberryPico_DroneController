#include "timer.h"
#include "attitude.h"
#include "servo.h"
#include "pulse.h"
#include "joystick.h"
#include "motors.h"
#include "cbit.h"
#include <stdio.h>


repeating_timer_t* cpu0_timer;
repeating_timer_t* cpu1_timer;


bool CPU0_TIMER_Init(int frequency_hz)
{
    cpu0_timer = new repeating_timer_t();

    int period_us = SECONDS_TO_US / frequency_hz;

    // Use default alarm pool for this core
    return add_repeating_timer_us(-period_us, CPU0_TIMER_Loop, NULL, cpu0_timer);
}

// Note : for this function to work it needs to be called from CPU1, othrerwise it will create another alarm pool whose callbacks will be executed on CPU0
bool CPU1_TIMER_Init(int frequency_hz)
{
    cpu1_timer = new repeating_timer_t();

    alarm_pool_t* pool = alarm_pool_create(1, 2);
    int period_us = SECONDS_TO_US / frequency_hz;
    return alarm_pool_add_repeating_timer_us(pool, -period_us, CPU1_TIMER_Loop, NULL, cpu1_timer);
}



bool CPU0_TIMER_Loop(repeating_timer_t* timer)
{
    JOYSTICK_Handler();
    MOTORS_Handler();
    CBIT_Handler();

    return true;
}


bool CPU1_TIMER_Loop(repeating_timer_t* timer)
{
    ATTITUDE_Handler();

    return true;
}