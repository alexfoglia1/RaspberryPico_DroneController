#include "imu_interface.h"
#include "user.h"

#include <hardware/gpio.h>

ImuInterface::ImuInterface()
{

}


bool ImuInterface::begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin)
{
    return true;
}