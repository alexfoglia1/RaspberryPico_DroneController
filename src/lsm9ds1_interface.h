#ifndef LSM9DS1_INTERFACE
#define LSM9DS1_INTERFACE

#include "imu_interface.h"
#include "LSM9DS1.h"

class LSM9DS1Interface : public ImuInterface
{

public:

    LSM9DS1Interface();
    
    bool begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin) override;
    bool haveMagneticField() override;
    bool haveAbsoluteOrientation() override;
    void getGyro(float* x, float* y, float* z) override;
    void getAccel(float* x, float* y, float* z) override;
    void getMagneticField(float* x, float* y, float* z) override;
    void getAbsoluteOrientation(float* roll, float* pitch, float* yaw) override;

private:
    LSM9DS1 _lsm9ds1;

};


#endif //LSM9DS1_INTERFACE