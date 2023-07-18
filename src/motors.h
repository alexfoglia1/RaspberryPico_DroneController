#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_MIN_SIGNAL    1000
#define MOTOR_MAX_SIGNAL    2000

#include <stdint.h>
#include "servo.h"

extern Servo motor1;
extern Servo motor2;
extern Servo motor3;
extern Servo motor4;

void MOTORS_Init();
void MOTORS_Handler();

#endif //MOTORS_H