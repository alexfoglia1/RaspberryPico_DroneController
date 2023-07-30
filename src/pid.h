#ifndef PID_H
#define PID_H

typedef struct
{
    float   ysp;
    float   y;
    float   ykm1;
    float   error;
    float   P;
    float   I;
    float   D;
    float   u;
} PID_CONTROL_TAG;

float pid_controller(PID_CONTROL_TAG* pid, float* gain, float ysp, float y);
void  pid_reset(PID_CONTROL_TAG* pid);

#endif //PID_H
