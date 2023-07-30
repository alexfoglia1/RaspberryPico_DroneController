#include "joystick.h"
#include "user.h"
#include "cbit.h"
#include <pico/float.h>
#include <pico/time.h>

float JOYSTICK_Roll;
float JOYSTICK_Pitch;
uint32_t JOYSTICK_Throttle;
bool  JOYSTICK_MotorsArmed;
bool  JOYSTICK_Timeout;

Pulse roll_signal(CHANNEL_ROLL);
Pulse pitch_signal(CHANNEL_PITCH);
Pulse throttle_signal(CHANNEL_THROTTLE);
Pulse armed_signal(CHANNEL_ARMED);

static float min_roll;
static float max_roll;
static float min_pitch;
static float max_pitch;

static float dead_center(float val)
{
    if (fabsf(val) < JS_DEAD_CENTER_ZONE_DEGREES)
    {
        return 0.0f;
    }
    else
    {
        return val;
    }
}

void JOYSTICK_Init(float min_r, float max_r, float min_p, float max_p)
{
    JOYSTICK_Roll = 0.0f;
    JOYSTICK_Pitch = 0.0f;
    JOYSTICK_Throttle = 0.0f;
    JOYSTICK_MotorsArmed = false;
    JOYSTICK_Timeout = false;

    min_roll = saturate(min_r, -360.f, 360.f);
    max_roll = saturate(max_r, -360.f, 360.f);
    min_pitch = saturate(min_p, -360.f, 360.f);
    max_pitch = saturate(max_p, -360.f, 360.f);

    roll_signal.attach();
    pitch_signal.attach();
    throttle_signal.attach();
    armed_signal.attach();
}


void JOYSTICK_Handler()
{
    uint64_t cur_t_us = time_us_64();

    uint32_t roll_signal_value = roll_signal.pulseIn();
    uint32_t pitch_signal_value = pitch_signal.pulseIn();
    uint32_t throttle_signal_value = throttle_signal.pulseIn();
    uint32_t armed_signal_value = armed_signal.pulseIn();

    JOYSTICK_Roll = dead_center(to_range(roll_signal_value, RADIO_MIN_SIGNAL_ROLL, RADIO_MAX_SIGNAL_ROLL, min_roll, max_roll));
    JOYSTICK_Pitch = -dead_center(to_range(pitch_signal_value, RADIO_MIN_SIGNAL_PITCH, RADIO_MAX_SIGNAL_PITCH, min_pitch, max_pitch));
    JOYSTICK_Throttle = throttle_signal_value;
    JOYSTICK_MotorsArmed = (armed_signal_value > ((RADIO_MAX_SIGNAL + RADIO_MIN_SIGNAL) / 2));

    /** Check failure **/
    uint64_t lastRollRise_t_us = roll_signal.lastRise_us();
    uint64_t lastPitchRise_t_us = pitch_signal.lastRise_us();
    uint64_t lastThrottleise_t_us = throttle_signal.lastRise_us();
    uint64_t lastArmedRise_t_us = armed_signal.lastRise_us();

    JOYSTICK_Timeout =  ((cur_t_us - lastRollRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastRollRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastRollRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastRollRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS));
                        
    if (JOYSTICK_Timeout)
    {
        JOYSTICK_MotorsArmed = false;
    }

}