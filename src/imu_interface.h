#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <hardware/i2c.h>

class ImuInterface
{

public:
    ImuInterface();
    
    virtual bool begin(i2c_inst_t* i2c, int sdaPin, int sclPin);

    virtual bool haveMagneticField() { return false; }
    virtual bool haveAbsoluteOrientation() { return false; }
    
    virtual void getGyro(float* x, float* y, float* z) {};
    virtual void getAccel(float* x, float* y, float* z) {};
    virtual void getMagneticField(float* x, float* y, float* z) {};
    virtual void getAbsoluteOrientation(float* roll, float* pitch, float* yaw) {};


};

#endif //IMU_INTERFACE_H