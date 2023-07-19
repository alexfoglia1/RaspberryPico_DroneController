#include "user.h"
#include "timer.h"
#include "attitude.h"
#include "joystick.h"
#include "motors.h"
#include "maint.h"
#include <stdio.h>

const int MOTOR_ARM_THRESHOLD = 100;

const float MIN_ROLL_DEGREES = -5.0f;
const float MAX_ROLL_DEGREES = 5.0f;
const float MIN_PITCH_DEGREES = -5.0f;
const float MAX_PITCH_DEGREES = 5.0;


int main()
{
    InitBoard();

    ATTITUDE_Init();

    JOYSTICK_Init(MIN_ROLL_DEGREES,
                  MAX_ROLL_DEGREES,
                  MIN_PITCH_DEGREES,
                  MAX_PITCH_DEGREES,
                  MOTOR_MIN_SIGNAL + MOTOR_ARM_THRESHOLD,
                  MOTOR_MAX_SIGNAL);

    MOTORS_Init();

    TIMER_Init(CTRL_LOOP_FREQUENCY_HZ);

    while(1)
    {
        MAINT_Handler();
    }
    return 0;
}