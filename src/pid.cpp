#include "pid.h"
#include "maint.h"

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
        pid->integral += (pid->error * gain[int(MAINT_PID_PARAM::PID_KT)]);
    }
    
    pid->derivative =  ((pid->error - ftmp) / gain[int(MAINT_PID_PARAM::PID_KT)]);
    pid->derivative2 = ((pid->derivative - derivative_km1) / gain[int(MAINT_PID_PARAM::PID_KT)]);

    // Compute P,I,D terms
    float KP = gain[int(MAINT_PID_PARAM::PID_KP)];
    float KI = gain[int(MAINT_PID_PARAM::PID_KI)];
    float KD = gain[int(MAINT_PID_PARAM::PID_AD)];
    float KD2 = gain[int(MAINT_PID_PARAM::PID_BD)];

    pid->P = KP * pid->error;
    pid->I = KI * pid->integral;
    pid->D = KD * pid->derivative;
    pid->D2 = KD2 * pid->derivative2;

    // Compute pid output
    float PID = pid->P + pid->I + pid->D + pid->D2;
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
    pid->D2 = 0.0f;
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->derivative2 = 0.0f;
    pid->output = 0.0f;
    pid->sat_flag = false;
}