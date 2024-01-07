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

static uint16_t roll_signal_value;
static uint16_t pitch_signal_value;
static uint16_t throttle_signal_value;
static uint16_t armed_signal_value;

static rx_throttle_status js_rx_status;
static bool armed_history[DEBOUNCE_WINDOW_LEN];
static uint16_t armed_history_ll;


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


static bool debounce_armed_signal()
{
    bool current_armed_status = (armed_signal_value > ((RADIO_MAX_SIGNAL + RADIO_MIN_SIGNAL) / 2));
    if (armed_history_ll < DEBOUNCE_WINDOW_LEN)
    {
        armed_history[armed_history_ll] = current_armed_status;
        armed_history_ll += 1;

        return JOYSTICK_MotorsArmed;
    }
    else
    {
        for (int i = 1; i < DEBOUNCE_WINDOW_LEN; i++)
        {
            armed_history[i - 1] = armed_history[i];
        }
        armed_history[DEBOUNCE_WINDOW_LEN - 1] = current_armed_status;
    }

    int hist_sum = 0;
    for (int i = 0; i < DEBOUNCE_WINDOW_LEN; i++)
    {
        hist_sum += (int) armed_history[i];
    }

    if (hist_sum == 0) return false;
    if (hist_sum == DEBOUNCE_WINDOW_LEN) return true;
    return JOYSTICK_MotorsArmed;
}


static void safe_disarm()
{
    JOYSTICK_Roll = (max_roll + min_roll) / 2.0f;
    JOYSTICK_Pitch = (max_pitch + min_pitch) / 2.0f;

    if (JOYSTICK_Timeout && js_rx_status == WAIT_ANY_SIGNAL)
    {
        // Radio link loss while UAS is airborne, descend
        js_rx_status = WAIT_ANY_SIGNAL;
        JOYSTICK_MotorsArmed = true;
        JOYSTICK_Throttle = MAINT_ThrottleParams[int(MAINT_THROTTLE_PARAM::DESCEND)];
    }
    else
    {
        // UAS is not airborne or it is a disarm command, disarm motors
        js_rx_status = WAIT_HALF_SIGNAL;
        JOYSTICK_MotorsArmed = false;
        JOYSTICK_Throttle = RADIO_MIN_SIGNAL;
    }
}


static uint16_t get_throttle_cmd(uint16_t radio_signal)
{
    float signal_percentage = to_range(radio_signal, RADIO_MIN_SIGNAL_THROTTLE, RADIO_MAX_SIGNAL_THROTTLE, 0.0f, 1.0f);
    uint16_t ret = RADIO_MIN_SIGNAL;

    switch (js_rx_status)
    {
        case WAIT_HALF_SIGNAL:
        {
            if (signal_percentage >= DESCEND_PERCENTAGE_THRESHOLD && signal_percentage < CLIMB_PERCENTAGE_THRESHOLD)
            {
                js_rx_status = WAIT_TAKEOFF_SIGNAL;
                ret = RADIO_MIN_SIGNAL;
            }

            break;
        }
        case WAIT_TAKEOFF_SIGNAL:
        {
            if (signal_percentage >= CLIMB_PERCENTAGE_THRESHOLD)
            {
                js_rx_status = WAIT_ANY_SIGNAL;
                ret = MAINT_ThrottleParams[int(MAINT_THROTTLE_PARAM::CLIMB)];
            }

            break;
        }
        case WAIT_ANY_SIGNAL:
        {
            if (signal_percentage < DESCEND_PERCENTAGE_THRESHOLD)
            {
                js_rx_status = WAIT_ANY_SIGNAL;
                ret = MAINT_ThrottleParams[int(MAINT_THROTTLE_PARAM::DESCEND)];
            }
            else if (signal_percentage >= DESCEND_PERCENTAGE_THRESHOLD && signal_percentage < CLIMB_PERCENTAGE_THRESHOLD)
            {
                js_rx_status = WAIT_ANY_SIGNAL;
                ret = MAINT_ThrottleParams[int(MAINT_THROTTLE_PARAM::HOVERING)];
            }
            else
            {
                js_rx_status = WAIT_ANY_SIGNAL;
                ret = MAINT_ThrottleParams[int(MAINT_THROTTLE_PARAM::CLIMB)];
            }
            break;
        }
    }

    return ret;
}


void JOYSTICK_Init(float min_r, float max_r, float min_p, float max_p)
{
    JOYSTICK_Roll = 0.0f;
    JOYSTICK_Pitch = 0.0f;
    JOYSTICK_Throttle = RADIO_MIN_SIGNAL;

    js_rx_status = WAIT_HALF_SIGNAL;

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

    armed_history_ll = 0;
}


void JOYSTICK_Handler()
{
    uint64_t cur_t_us = time_us_64();

    if (MAINT_IsOverridingRadio())
    {
        MAINT_ReadOverridenSignals(&armed_signal_value, &roll_signal_value, &pitch_signal_value, &throttle_signal_value);
    }
    else
    {
        roll_signal_value = *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::ROLL)][int(MAINT_JS_PARAM::ALPHA)]) * roll_signal_value +
                                *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::ROLL)][int(MAINT_JS_PARAM::BETA)]) * (uint16_t) (roll_signal.pulseIn() & 0xFFFF);

        pitch_signal_value = *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::PITCH)][int(MAINT_JS_PARAM::ALPHA)]) * pitch_signal_value +
                                *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::PITCH)][int(MAINT_JS_PARAM::BETA)]) * (uint16_t) (pitch_signal.pulseIn() & 0xFFFF);

        throttle_signal_value = *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::THROTTLE)][int(MAINT_JS_PARAM::ALPHA)]) * throttle_signal_value +
                                    *reinterpret_cast<float*>(&MAINT_JoystickParameters[int(JOYSTICK_CHANNEL::THROTTLE)][int(MAINT_JS_PARAM::BETA)]) * (uint16_t) (throttle_signal.pulseIn() & 0xFFFF);

        armed_signal_value = (uint16_t) (armed_signal.pulseIn() & 0xFFFF);
    }

    
    if (MAINT_IsOverridingRadio())
    {
        JOYSTICK_Roll = to_range(roll_signal_value, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, min_roll, max_roll);
        JOYSTICK_Pitch = -to_range(pitch_signal_value, RADIO_MIN_SIGNAL, RADIO_MAX_SIGNAL, min_pitch, max_pitch);
        JOYSTICK_MotorsArmed = (armed_signal_value > ((RADIO_MAX_SIGNAL + RADIO_MIN_SIGNAL) / 2));
    }
    else
    {
        JOYSTICK_Roll = dead_center(to_range(roll_signal_value, RADIO_MIN_SIGNAL_ROLL, RADIO_MAX_SIGNAL_ROLL, min_roll, max_roll));
        JOYSTICK_Pitch = -dead_center(to_range(pitch_signal_value, RADIO_MIN_SIGNAL_PITCH, RADIO_MAX_SIGNAL_PITCH, min_pitch, max_pitch));
        JOYSTICK_MotorsArmed = debounce_armed_signal();
    }

    JOYSTICK_Throttle = get_throttle_cmd(throttle_signal_value);
    
    /** Check failure **/
    uint64_t lastRollRise_t_us = roll_signal.lastRise_us();
    uint64_t lastPitchRise_t_us = pitch_signal.lastRise_us();
    uint64_t lastThrottleRise_t_us = throttle_signal.lastRise_us();
    uint64_t lastArmedRise_t_us = armed_signal.lastRise_us();

    JOYSTICK_Timeout =  ((cur_t_us - lastRollRise_t_us) > (JS_TIMEOUT_MILLIS * MILLISECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastPitchRise_t_us) > (JS_TIMEOUT_MILLIS * MILLISECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastThrottleRise_t_us) > (JS_TIMEOUT_MILLIS * MILLISECONDS_TO_MICROSECONDS)) ||
                        ((cur_t_us - lastArmedRise_t_us) > (JS_TIMEOUT_MILLIS * MILLISECONDS_TO_MICROSECONDS));
    
    if (MAINT_IsOverridingRadio())
    {
        JOYSTICK_Timeout = !MAINT_IsPresent();
    }
                    
    if (JOYSTICK_Timeout)
    {
        JOYSTICK_MotorsArmed = false;
    }

    if (!JOYSTICK_MotorsArmed)
    {
        safe_disarm();
    }
}
