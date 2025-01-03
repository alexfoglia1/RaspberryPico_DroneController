#include "pid.h"
#include "maint.h"
#include "user.h"

static const float PID_DT = 1.0f/static_cast<float>(CTRL_LOOP_FREQUENCY_HZ);

static float minMax(float val, float min, float max, bool* in_sat)
{
    if (val < min)
    {
        *in_sat = true;
        return min;
    }

    if (val > max)
    {
        *in_sat = true;
        return max;
    }

    *in_sat = false;
    return val;
}


float pid_controller(PID_CONTROL_TAG* pid, float* gain, float ysp, float y)
{
    float ftmp = 0.0f;
    float derivative_km1 = pid->derivative;
    bool  btmp = false;

    // Update actual values
    ftmp = pid->error;
    pid->error = ysp - y;
    //
    // Compute integral and derivative of error signal
    if (!pid->sat_flag)
    {
        pid->integral += (pid->error * PID_DT);
    }
    
    pid->derivative =  ((pid->error - ftmp) / PID_DT);

    // Compute P,I,D terms
    float KP = gain[int(MAINT_PID_PARAM::PID_KP)];
    float KI = gain[int(MAINT_PID_PARAM::PID_KI)];
    float KD = gain[int(MAINT_PID_PARAM::PID_KD)];

    pid->P = KP * pid->error;
    pid->I = KI * pid->integral;
    pid->D = KD * pid->derivative;

    // Compute pid output
    float PID = pid->P + pid->I + pid->D;
    float SAT = gain[int(MAINT_PID_PARAM::PID_SAT)];

    pid->output = minMax(PID, -SAT, SAT, &btmp);
    pid->sat_flag = btmp;

    return pid->output;
}


void pid_reset(PID_CONTROL_TAG* pid)
{
    pid->P = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->sat_flag = false;
}