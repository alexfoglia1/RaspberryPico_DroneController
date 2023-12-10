#include "imu_interface.h"
#include "user.h"

#include <hardware/gpio.h>

ImuInterface::ImuInterface()
{

}


bool ImuInterface::begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin)
{
    i2c_init(i2c_channel, I2C_FREQUENCY_HZ);

    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    return true;
}