#include "cbit.h"
#include "maint.h"
#include "joystick.h"

CBIT_TAG cbit_status;


void CBIT_Init()
{
    cbit_status.Dword = 0;
}


void CBIT_Handler()
{
    // Perform cbit checks here
    cbit_status.Bits.js_timeout    = JOYSTICK_Timeout ? 1 : 0;
    cbit_status.Bits.maint_timeout = !MAINT_IsPresent() ? 1 : 0;
}


void CBIT_Set_fail_code(uint32_t fail_code, bool active)
{
    if (active)
    {
        cbit_status.Dword |= fail_code;
    }
    else
    {
        cbit_status.Dword &= ~fail_code;
    }
}


bool CBIT_Fails_live(uint32_t fail_code)
{
    return (cbit_status.Dword & fail_code) != 0;
}