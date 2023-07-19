#include "pid.h"


static float minMax(float val, float min, float max)
{
    return (val < min) ? min :
           (val > max) ? max :
           val;
}


float pid_controller(PID_CONTROL_TAG* pid, float* gain, float ysp, float y)
{
    float ftmp;

    //  updates actual values
    pid->ysp = ysp;
    pid->y = y;
    pid->error = pid->ysp - pid->y;
    //
    pid->P = pid->error * gain[PID_KP];
    pid->D = pid->D * gain[PID_AD] - (pid->y - pid->ykm1) * gain[PID_BD];
    ftmp = pid->P + pid->I + pid->D;
    pid->u = minMax(ftmp, -gain[PID_SAT], gain[PID_SAT]);
    //
    pid->I += pid->error * gain[PID_KI];
    pid->I += (pid->u - ftmp) * gain[PID_KT];
    //
    pid->ykm1 = pid->y;
    //
    return pid->u;
}


void pid_reset(PID_CONTROL_TAG* pid)
{
    pid->error = 0.0f;
    pid->I = 0.0f;
    pid->D = 0.0f;
    pid->u = 0.0f;
}