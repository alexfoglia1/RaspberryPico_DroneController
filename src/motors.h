#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_MIN_SIGNAL    1000
#define MOTOR_MAX_SIGNAL    2000

#include <stdint.h>
#include "servo.h"
#include "pid.h"

extern Servo motor1;
extern Servo motor2;
extern Servo motor3;
extern Servo motor4;
extern PID_CONTROL_TAG pid_roll;
extern PID_CONTROL_TAG pid_pitch;
extern PID_CONTROL_TAG pid_yaw;

#ifdef __CONTROL_RPM__
extern uint32_t MOTOR_Throttle;
#else
#define MOTOR_ARMED_THRESHOLD 250
#endif

void MOTORS_Init();
void MOTORS_Handler();

#endif //MOTORS_H