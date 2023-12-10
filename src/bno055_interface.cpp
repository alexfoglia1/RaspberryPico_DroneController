#include "bno055_interface.h"

BNO055Interface::BNO055Interface()
{

}
    
bool BNO055Interface::begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin)
{
    ImuInterface::begin(i2c_channel, sdaPin, sclPin);
    
    return _bno055.initialize(equipment_handlers::BNO055_IMU::BNO055_Primary_Address, i2c_channel);
}


bool BNO055Interface::haveMagneticField()
{
    return true;
}


bool BNO055Interface::haveAbsoluteOrientation()
{
    return true;
}


void BNO055Interface::getGyro(float* x, float* y, float* z)
{
    equipment_handlers::BNO055_IMU::AngularVelocity velData;
    _bno055.readSensorData(velData);

    *x = velData.X;
    *y = velData.Y;
    *z = velData.Z;
}


void BNO055Interface::getAccel(float* x, float* y, float* z)
{
    equipment_handlers::BNO055_IMU::Acceleration accData;
    _bno055.readSensorData(accData);

    *x = accData.X;
    *y = accData.Y;
    *z = accData.Z;
}


void BNO055Interface::getMagneticField(float* x, float* y, float* z)
{
    equipment_handlers::BNO055_IMU::MagField magData;
    _bno055.readSensorData(magData);

    *x = magData.X;
    *y = magData.Y;
    *z = magData.Z;
}


void BNO055Interface::getAbsoluteOrientation(float* roll, float* pitch, float* yaw)
{
    equipment_handlers::BNO055_IMU::EulerOrientation oriData;
    _bno055.readFusedData(oriData);

    *roll = oriData.X;
    *pitch = oriData.Y;
    *yaw = oriData.Z;
}