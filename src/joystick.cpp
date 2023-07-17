#include "joystick.h"
#include "user.h"
#include "pulse.h"
#include <pico/float.h>

float JOYSTICK_Roll;
float JOYSTICK_Pitch;
float JOYSTICK_Throttle;
bool  JOYSTICK_MotorsArmed;

static Pulse roll_signal(CHANNEL_ROLL);
static Pulse pitch_signal(CHANNEL_PITCH);
static Pulse throttle_signal(CHANNEL_THROTTLE);
static Pulse armed_signal(CHANNEL_ARMED);

static float min_roll;
static float max_roll;
static float min_pitch;
static float max_pitch;
static float min_throttle;
static float max_throttle;

static float saturate(float val, float min, float max)
{
    return (val < min) ? min :
           (val > max) ? max :
           val;
}

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

static float to_range(float in, float in_min, float in_max, float out_min, float out_max)
{
    in = saturate(in, in_min, in_max);
    float in_percentage = (in - in_min) / (in_max - in_min);
    
    return out_min + (in_percentage) * (out_max - out_min);
}

void JOYSTICK_Init(float min_r, float max_r, float min_p, float max_p, float min_t, float max_t)
{
    JOYSTICK_Roll = 0.0f;
    JOYSTICK_Pitch = 0.0f;
    JOYSTICK_Throttle = 0.0f;
    JOYSTICK_MotorsArmed = false;

    min_roll = saturate(min_r, -360.f, 360.f);
    max_roll = saturate(max_r, -360.f, 360.f);
    min_pitch = saturate(min_p, -360.f, 360.f);
    max_pitch = saturate(max_p, -360.f, 360.f);
    min_throttle = saturate(min_t, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL);
    max_throttle = saturate(max_t, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL);

    roll_signal.attach();
    pitch_signal.attach();
    throttle_signal.attach();
    armed_signal.attach();
}


void JOYSTICK_Handler()
{
    uint32_t roll_signal_value = roll_signal.pulseIn();
    uint32_t pitch_signal_value = pitch_signal.pulseIn();
    uint32_t throttle_signal_value = throttle_signal.pulseIn();
    uint32_t armed_signal_value = armed_signal.pulseIn();

    JOYSTICK_Roll = dead_center(to_range(roll_signal_value, RADIO_MIN_SIGNAL_ROLL, RADIO_MAX_SIGNAL_ROLL, min_roll, max_roll));
    JOYSTICK_Pitch = dead_center(to_range(pitch_signal_value, RADIO_MIN_SIGNAL_PITCH, RADIO_MAX_SIGNAL_PITCH, min_pitch, max_pitch));
    JOYSTICK_Throttle = dead_center(to_range(throttle_signal_value, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, min_throttle, max_throttle));
    JOYSTICK_MotorsArmed = (armed_signal_value > ((RADIO_MAX_SIGNAL + RADIO_MIN_SIGNAL) / 2));
}