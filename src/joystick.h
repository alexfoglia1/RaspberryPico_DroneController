#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "pulse.h"

#define RADIO_MIN_SIGNAL 1000
#define RADIO_MAX_SIGNAL 2000
#define RADIO_MIN_SIGNAL_ROLL 1046
#define RADIO_MAX_SIGNAL_ROLL 1920
#define RADIO_MIN_SIGNAL_PITCH 1047
#define RADIO_MAX_SIGNAL_PITCH 1927

#define JS_DEAD_CENTER_ZONE_DEGREES 1.0f

extern float JOYSTICK_Roll;
extern float JOYSTICK_Pitch;
extern float JOYSTICK_Throttle;
extern bool  JOYSTICK_MotorsArmed;
extern bool  JOYSTICK_Timeout;

extern Pulse roll_signal;
extern Pulse pitch_signal;
extern Pulse throttle_signal;
extern Pulse armed_signal;

void JOYSTICK_Init(float min_roll, float max_roll, float min_pitch, float max_pitch, float min_throttle, float max_throttle);
void JOYSTICK_Handler();

#endif //JOYSTICK_H