#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "pulse.h"

#define RADIO_MIN_SIGNAL 1000
#define RADIO_MAX_SIGNAL 2000
#define RADIO_MIN_SIGNAL_THROTTLE 1046
#define RADIO_MAX_SIGNAL_THROTTLE 1920
#define RADIO_MIN_SIGNAL_ROLL 1000
#define RADIO_MAX_SIGNAL_ROLL 2000
#define RADIO_MIN_SIGNAL_PITCH 1046
#define RADIO_MAX_SIGNAL_PITCH 1935

#define JS_DEAD_CENTER_ZONE_DEGREES 1.0f

#define DESCEND_PERCENTAGE_THRESHOLD 0.33333334f
#define CLIMB_PERCENTAGE_THRESHOLD 0.6666667f

#define DEBOUNCE_WINDOW_LEN 10 // At least 10 period of pwm with equal duty cycle, timeout occurs before because it elapses after one period of pwm with no rising edge detected
                               // Hence fail-safe due to radio link loss has the priority to disarm motors


#ifdef __JOYMODE_2__
typedef enum
{
    WAIT_HALF_SIGNAL,
    WAIT_TAKEOFF_SIGNAL,
    WAIT_ANY_SIGNAL
} rx_throttle_status;
#endif


extern float JOYSTICK_Roll;
extern float JOYSTICK_Pitch;
extern uint32_t JOYSTICK_Throttle;
extern bool  JOYSTICK_MotorsArmed;
extern bool  JOYSTICK_Timeout;

extern Pulse roll_signal;
extern Pulse pitch_signal;
extern Pulse throttle_signal;
extern Pulse armed_signal;

void JOYSTICK_Init(float min_roll, float max_roll, float min_pitch, float max_pitch);
void JOYSTICK_Handler();

#endif //JOYSTICK_H