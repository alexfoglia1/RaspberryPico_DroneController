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

typedef enum
{
    PID_KP,
    PID_KI,
    PID_KT,
    PID_SAT,
    PID_AD,
    PID_BD,
    PID_PARAMS_SIZE
} PID_GAIN_ORDER;

float pid_controller(PID_CONTROL_TAG* pid, float* gain, float ysp, float y);
void  pid_reset(PID_CONTROL_TAG* pid);

#endif //PID_H
