#include "mpu6050_interface.h"


MPU6050Interface::MPU6050Interface()
{

}


bool MPU6050Interface::begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin)
{
    ImuInterface::begin(i2c_channel, sdaPin, sclPin);
    
    return _mpu6050.init();
}


void MPU6050Interface::getAccel(float* x, float* y, float* z)
{
    // MPU6050 inverted axis
    _mpu6050.readAccel(y, x, z);
}


void MPU6050Interface::getGyro(float* x, float* y, float* z)
{
    // MPU6050 inverted axis
    _mpu6050.readGyro(y, x, z);
}


void MPU6050Interface::getMagneticField(float* x, float* y, float* z)
{
    // Unsupported
    *x = 0;
    *y = 0;
    *z = 0;
}


void MPU6050Interface::getAbsoluteOrientation(float* roll, float* pitch, float* yaw)
{
    // Unsupported
    *roll = 0;
    *pitch = 0;
    *yaw = 0;
}


bool MPU6050Interface::haveMagneticField()
{
    return false;
}


bool MPU6050Interface::haveAbsoluteOrientation()
{
    return false;
}