#include "user.h"
#include "timer.h"
#include "attitude.h"
#include "joystick.h"
#include "motors.h"
#include "maint.h"
#include "cbit.h"
#include <stdio.h>
#include <pico/multicore.h>

const float MIN_ROLL_DEGREES = -5.0f;
const float MAX_ROLL_DEGREES = 5.0f;
const float MIN_PITCH_DEGREES = -5.0f;
const float MAX_PITCH_DEGREES = 5.0;

#define CPU_CHECK 0xB0BAF377 // A long time ago, in a galaxy far far away this drone have an hyperguide


void __cpu1_entry_point__()
{
    CBIT_TAG fail_code;
    fail_code.Dword = 0;
    fail_code.Bits.cpu1_launch_failure = 1;

    multicore_fifo_push_blocking(CPU_CHECK);
    uint32_t cpu_check = multicore_fifo_pop_blocking();

    if (cpu_check != CPU_CHECK)
    {
        /** Set fail code **/
        CBIT_Set_fail_code(fail_code.Dword, true);
    }
    else
    {
        CBIT_Set_fail_code(fail_code.Dword, false);

        /** Start attitude estimation timer loop **/
        fail_code.Dword = 0;
        fail_code.Bits.cpu1_timer_failure = 1;

        if (!CPU1_TIMER_Init(CTRL_LOOP_FREQUENCY_HZ))
        {
            CBIT_Set_fail_code(fail_code.Dword, true);
        }
        else
        {
            CBIT_Set_fail_code(fail_code.Dword, false);
        }
    }

    while (1)
    {
        tight_loop_contents();
    }
}


int main()
{
    CBIT_TAG fail_code;
    fail_code.Dword = 0;
    fail_code.Bits.imu_failure = 1;

    /** Initialize gpio directions **/
    InitBoard();

    MOTORS_Init();
    MAINT_Init();
    CBIT_Init();

    /** Initialize application **/
    if (!ATTITUDE_Init())
    {
        CBIT_Set_fail_code(fail_code.Dword, true);
    }
    else
    {
        CBIT_Set_fail_code(fail_code.Dword, false);
    }


    JOYSTICK_Init(MIN_ROLL_DEGREES,
                  MAX_ROLL_DEGREES,
                  MIN_PITCH_DEGREES,
                  MAX_PITCH_DEGREES);

    /** Launch attitude producer timer on CPU1 **/
    multicore_launch_core1(__cpu1_entry_point__);
    /** Wait for it to start **/
    uint32_t cpu_check = multicore_fifo_pop_blocking();

    fail_code.Dword = 0;
    fail_code.Bits.cpu0_launch_failure = 1;
    if (cpu_check != CPU_CHECK)
    {
        /** Set fail code **/
        CBIT_Set_fail_code(fail_code.Dword, true);
    }
    else
    {
        multicore_fifo_push_blocking(CPU_CHECK);
        CBIT_Set_fail_code(fail_code.Dword, false);

        /** Attitude producer is started on CPU1, can start consuming **/
        fail_code.Dword = 0;
        fail_code.Bits.cpu0_timer_failure = 1;
        if (!CPU0_TIMER_Init(CTRL_LOOP_FREQUENCY_HZ))
        {
            CBIT_Set_fail_code(fail_code.Dword, true);
        }
        else
        {
            CBIT_Set_fail_code(fail_code.Dword, false);
        }
    }

    while(1)
    {
        MAINT_Handler();
    }
    
    return 0;
}