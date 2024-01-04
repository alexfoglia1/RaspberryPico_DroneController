#include "joystick.h"
#include "user.h"
#include "cbit.h"
#include "maint.h"

#include <pico/float.h>
#include <pico/time.h>

float JOYSTICK_Roll;
float JOYSTICK_Pitch;
uint32_t JOYSTICK_Throttle;
bool  JOYSTICK_MotorsArmed;
bool  JOYSTICK_Timeout;

Pulse roll_signal(CHANNEL_ROLL_GPIO);
Pulse pitch_signal(CHANNEL_PITCH_GPIO);
Pulse throttle_signal(CHANNEL_THROTTLE_GPIO);
Pulse armed_signal(CHANNEL_ARMED_GPIO);

static float min_roll;
static float max_roll;
static float min_pitch;
static float max_pitch;

static uint32_t roll_signal_value;
static uint32_t pitch_signal_value;
static uint32_t throttle_signal_value;
static uint32_t armed_signal_value;


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

    min_roll = saturate(min_r, -180.f, 180.f);
    max_roll = saturate(max_r, -180.f, 180.f);
    min_pitch = saturate(min_p, -180.f, 180.f);
    max_pitch = saturate(max_p, -180.f, 180.f);

    roll_signal.attach();
    pitch_signal.attach();
    throttle_signal.attach();
    armed_signal.attach();

    roll_signal_value = 0;
    pitch_signal_value = 0;
    throttle_signal_value = 0;
    armed_signal_value = 0;
}


void JOYSTICK_Handler()
{
    uint64_t cur_t_us = time_us_64();


    roll_signal_value = *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::ROLL)][int(MAINT_JS_PARAM::ALPHA)]) * roll_signal_value +
                            *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::ROLL)][int(MAINT_JS_PARAM::BETA)]) * roll_signal.pulseIn();

    pitch_signal_value = *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::PITCH)][int(MAINT_JS_PARAM::ALPHA)]) * pitch_signal_value +
                            *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::PITCH)][int(MAINT_JS_PARAM::BETA)]) * pitch_signal.pulseIn();

    throttle_signal_value = *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::THROTTLE)][int(MAINT_JS_PARAM::ALPHA)]) * throttle_signal_value +
                                *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::THROTTLE)][int(MAINT_JS_PARAM::BETA)]) * throttle_signal.pulseIn();

    armed_signal_value = armed_signal.pulseIn();

    JOYSTICK_MotorsArmed = (armed_signal_value > ((RADIO_MAX_SIGNAL + RADIO_MIN_SIGNAL) / 2));
    if (JOYSTICK_MotorsArmed)
    {
        JOYSTICK_Roll = dead_center(to_range(roll_signal_value, RADIO_MIN_SIGNAL_ROLL, RADIO_MAX_SIGNAL_ROLL, min_roll, max_roll));
        JOYSTICK_Pitch = -dead_center(to_range(pitch_signal_value, RADIO_MIN_SIGNAL_PITCH, RADIO_MAX_SIGNAL_PITCH, min_pitch, max_pitch));
        JOYSTICK_Throttle = throttle_signal_value;
    }
    else
    {
        JOYSTICK_Roll = (max_roll + min_roll) / 2.0f;
        JOYSTICK_Pitch = (max_pitch + min_pitch) / 2.0f;
        JOYSTICK_Throttle = RADIO_MIN_SIGNAL;
    }

    /** Check failure **/
    uint64_t lastRollRise_t_us = roll_signal.lastRise_us();
    uint64_t lastPitchRise_t_us = pitch_signal.lastRise_us();
    uint64_t lastThrottleRise_t_us = throttle_signal.lastRise_us();
    uint64_t lastArmedRise_t_us = armed_signal.lastRise_us();

    JOYSTICK_Timeout =  ((cur_t_us - lastRollRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastPitchRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastThrottleRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastArmedRise_t_us) > (JS_TIMEOUT_S * SECONDS_TO_MICROSECONDS));
                    
    if (JOYSTICK_Timeout)
    {
        JOYSTICK_MotorsArmed = false;
    }
    
}
