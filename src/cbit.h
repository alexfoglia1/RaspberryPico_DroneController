#ifndef CBIT_H
#define CBIT_H

#include <stdint.h>

typedef union
{
    struct
    {
       uint32_t imu_failure          :1;
       uint32_t js_timeout           :1;
       uint32_t maint_timeout        :1;
       uint32_t cpu0_launch_failure  :1;
       uint32_t cpu1_launch_failure  :1;
       uint32_t cpu0_timer_failure   :1;
       uint32_t cpu1_timer_failure   :1;
       uint32_t                      :25;
    } Bits;
    uint8_t Bytes[4];
    uint32_t Dword;
} CBIT_TAG;


extern CBIT_TAG cbit_status;

void CBIT_Init();
void CBIT_Handler();
void CBIT_Set_fail_code(uint32_t fail_code, bool active);
bool CBIT_Fails_live(uint32_t fail_code);



#endif //CBIT_H
