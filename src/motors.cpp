#include "motors.h"
#include "servo.h"
#include "user.h"
#include "attitude.h"
#include "joystick.h"


static Servo motor1(MOTOR_1);
static Servo motor2(MOTOR_2);
static Servo motor3(MOTOR_3);
static Servo motor4(MOTOR_4);

uint32_t M1_Signal;
uint32_t M2_Signal;
uint32_t M3_Signal;
uint32_t M4_Signal;

void MOTORS_Init()
{
    M1_Signal = MOTOR_MIN_SIGNAL;
    M2_Signal = MOTOR_MIN_SIGNAL;
    M3_Signal = MOTOR_MIN_SIGNAL;
    M4_Signal = MOTOR_MIN_SIGNAL;

    motor1.attach();
    motor2.attach();
    motor3.attach();
    motor4.attach();    
}


void MOTORS_Handler()
{
    M1_Signal = MOTOR_MIN_SIGNAL;
    M2_Signal = MOTOR_MIN_SIGNAL;
    M3_Signal = MOTOR_MIN_SIGNAL;
    M4_Signal = MOTOR_MIN_SIGNAL;
    
    if (JOYSTICK_MotorsArmed)
    {
        //TODO : PID
        M1_Signal = JOYSTICK_Throttle;
        M2_Signal = JOYSTICK_Throttle;
        M3_Signal = JOYSTICK_Throttle;
        M4_Signal = JOYSTICK_Throttle;
    }

    motor1.writeMicroseconds(M1_Signal);
    motor2.writeMicroseconds(M2_Signal);
    motor3.writeMicroseconds(M3_Signal);
    motor4.writeMicroseconds(M4_Signal);
}