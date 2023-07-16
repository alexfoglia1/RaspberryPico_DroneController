#include "user.h"
#include "timer.h"
#include "attitude.h"
#include <stdio.h>


int main()
{
    InitBoard();

    if (!ATTITUDE_Init())
    {
        printf("Imu error\r\n");
        while(1);
    }

    if (!TIMER_Init(CTRL_LOOP_FREQUENCY_HZ))
    {
        // Handle timer failure
    }

    while(1)
    {
        printf("Orientation: %f %f %f\n", ATTITUDE_Roll, ATTITUDE_Pitch, ATTITUDE_Yaw);
    }
    return 0;
}