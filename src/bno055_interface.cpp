#include "bno055_interface.h"

#include <pico/float.h>

#define PI 3.14159265358979323846264338327950288419716939937510582f

static const float DEGREES_TO_RADIANS = PI/180.f;
static const float RADIANS_TO_DEGREES = 180.0f/PI;

BNO055Interface::BNO055Interface()
{
    _roll0 = 0;
    _pitch0 = 0;
}
    
bool BNO055Interface::begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin)
{
    ImuInterface::begin(i2c_channel, sdaPin, sclPin);
    
    bool success = _bno055.initialize(equipment_handlers::BNO055_IMU::BNO055_Primary_Address, i2c_channel);

    if (success)
    {
        for (int i = 0; i < 10; i++)
        {
            equipment_handlers::BNO055_IMU::EulerOrientation oriData;
            _bno055.readFusedData(oriData);

            _roll0 = oriData.Y > _roll0 ? oriData.Y : _roll0;
            _pitch0 = oriData.Z > _pitch0 ? oriData.Z : _pitch0;
            sleep_ms(500);
        }
    }

    return success;
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

    // Driver reads starting from heading (x, y, z is just the order of read not the relative axes)
    *yaw = atan2(sin(oriData.X * DEGREES_TO_RADIANS), cos(oriData.X * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
    *roll = atan2(sin((oriData.Y - _roll0) * DEGREES_TO_RADIANS), cos((oriData.Y - _roll0) * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
    *pitch = atan2(sin((oriData.Z - _pitch0) * DEGREES_TO_RADIANS), cos((oriData.Z - _pitch0) * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
}