#include "user.h"
#include "timer.h"
#include "attitude.h"
#include "joystick.h"
#include "motors.h"
#include <stdio.h>

const int MOTOR_ARM_THRESHOLD = 100;

const float MIN_ROLL_DEGREES = -5.0f;
const float MAX_ROLL_DEGREES = 5.0f;
const float MIN_PITCH_DEGREES = -5.0f;
const float MAX_PITCH_DEGREES = 5.0;


int main()
{
    InitBoard();

    //ATTITUDE_Init();

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
#ifdef __ASCI_MAINT__
        printf("ROLL(%f)\tPITCH(%f)\tYAW(%f)\t",
        ATTITUDE_Roll, ATTITUDE_Pitch, ATTITUDE_Yaw);

        printf("JS_ROLL(%f)\tJS_PITCH(%f)\tJS_THROTTLE(%f)\tJS_ARM(%d)\t",
        JOYSTICK_Roll, JOYSTICK_Pitch, JOYSTICK_Throttle, JOYSTICK_MotorsArmed);

        printf("M1(%d)\tM2(%d)\tM3(%d)\tM4(%d)\n",
        M1_Signal, M2_Signal, M3_Signal, M4_Signal);
#else
        MAINT_Manager();
#endif
    }
    return 0;
}