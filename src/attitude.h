#ifndef ATTITUDE_H
#define ATTITUDE_H

#define PI 3.14159265358979323846264338327950288419716939937510582f

typedef struct
{
    float raw_km1;
    float filt_km1;
    float raw_k;
    float filt_k;
} pt1_flt_tag;

extern float ATTITUDE_Roll;
extern float ATTITUDE_Pitch;
extern float ATTITUDE_Yaw;

bool ATTITUDE_Init(void);
void ATTITUDE_Handler(void);


#endif //ATTITUDE_H