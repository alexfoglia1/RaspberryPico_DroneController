#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_MIN_SIGNAL    1000
#define MOTOR_MAX_SIGNAL    2000

#include <stdint.h>

extern uint32_t M1_Signal;
extern uint32_t M2_Signal;
extern uint32_t M3_Signal;
extern uint32_t M4_Signal;

void MOTORS_Init();
void MOTORS_Handler();

#endif //MOTORS_H