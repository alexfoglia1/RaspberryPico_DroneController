#include "motors.h"
#include "user.h"
#include "attitude.h"
#include "joystick.h"


Servo motor1(MOTOR_1);
Servo motor2(MOTOR_2);
Servo motor3(MOTOR_3);
Servo motor4(MOTOR_4);


void MOTORS_Init()
{
    motor1.attach();
    motor2.attach();
    motor3.attach();
    motor4.attach();    
}


void MOTORS_Handler()
{
    uint32_t m1_signal = MOTOR_MIN_SIGNAL;
    uint32_t m2_signal = MOTOR_MIN_SIGNAL;
    uint32_t m3_signal = MOTOR_MIN_SIGNAL;
    uint32_t m4_signal = MOTOR_MIN_SIGNAL;
    
    if (JOYSTICK_MotorsArmed)
    {
        //TODO : PID
        m1_signal = JOYSTICK_Throttle;
        m2_signal = JOYSTICK_Throttle;
        m3_signal = JOYSTICK_Throttle;
        m4_signal = JOYSTICK_Throttle;
    }

    motor1.writeMicroseconds(m1_signal);
    motor2.writeMicroseconds(m2_signal);
    motor3.writeMicroseconds(m3_signal);
    motor4.writeMicroseconds(m4_signal);
}