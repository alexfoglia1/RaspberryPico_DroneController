#include "lsm9ds1_interface.h"


LSM9DS1Interface::LSM9DS1Interface()
{
    
}


bool LSM9DS1Interface::begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin)
{
    ImuInterface::begin(i2c_channel, sdaPin, sclPin);
    
    return _lsm9ds1.begin();
}


void LSM9DS1Interface::getAccel(float* x, float* y, float* z)
{
    _lsm9ds1.readAccel();
    *x = _lsm9ds1.calcAccel(_lsm9ds1.ax);
    *y = _lsm9ds1.calcAccel(_lsm9ds1.ay);
    *z = _lsm9ds1.calcAccel(_lsm9ds1.az);
}


void LSM9DS1Interface::getGyro(float* x, float* y, float* z)
{
    _lsm9ds1.readGyro();
    *x = _lsm9ds1.calcGyro(_lsm9ds1.gx);
    *y = _lsm9ds1.calcGyro(_lsm9ds1.gy);
    *z = _lsm9ds1.calcGyro(_lsm9ds1.gz);
}


void LSM9DS1Interface::getMagneticField(float* x, float* y, float* z)
{
    _lsm9ds1.readMag();
    *x = _lsm9ds1.calcMag(_lsm9ds1.mx);
    *y = _lsm9ds1.calcMag(_lsm9ds1.my);
    *z = _lsm9ds1.calcMag(_lsm9ds1.mz);
}


void LSM9DS1Interface::getAbsoluteOrientation(float* roll, float* pitch, float* yaw)
{
    // Unimplemented by hardware
    *roll = 0;
    *pitch = 0;
    *yaw = 0;
}


bool LSM9DS1Interface::haveMagneticField()
{
    return true;
}


bool LSM9DS1Interface::haveAbsoluteOrientation()
{
    return false;
}