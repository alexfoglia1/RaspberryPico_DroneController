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
extern pt1_flt_tag ax_flt_tag;
extern pt1_flt_tag ay_flt_tag;
extern pt1_flt_tag az_flt_tag;
extern pt1_flt_tag gx_flt_tag;
extern pt1_flt_tag gy_flt_tag;
extern pt1_flt_tag gz_flt_tag;
extern pt1_flt_tag mx_flt_tag;
extern pt1_flt_tag my_flt_tag;
extern pt1_flt_tag mz_flt_tag;
extern float ATTITUDE_Roll0;
extern float ATTITUDE_Pitch0;

bool ATTITUDE_Init();
void ATTITUDE_Calibrate(bool power_on);
void ATTITUDE_Handler();


#endif //ATTITUDE_H