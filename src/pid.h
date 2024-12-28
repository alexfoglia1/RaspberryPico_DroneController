#ifndef PID_H
#define PID_H

typedef struct
{
    float   error;
    float   integral;
    float   derivative;
    float   output;
    bool    sat_flag;

    float P;
    float I;
    float D;
} PID_CONTROL_TAG;

float pid_controller(PID_CONTROL_TAG* pid, float* gain, float ysp, float y);
void  pid_reset(PID_CONTROL_TAG* pid);

#endif //PID_H
