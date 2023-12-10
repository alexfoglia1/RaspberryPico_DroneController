#ifndef MPU6050_INTERFACE
#define MPU6050_INTERFACE

#include "imu_interface.h"
#include "mpu6050.h"

class MPU6050Interface : public ImuInterface
{

public:

    MPU6050Interface();
    
    bool begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin) override;
    bool haveMagneticField() override;
    bool haveAbsoluteOrientation() override;
    void getGyro(float* x, float* y, float* z) override;
    void getAccel(float* x, float* y, float* z) override;
    void getMagneticField(float* x, float* y, float* z) override;
    void getAbsoluteOrientation(float* roll, float* pitch, float* yaw) override;

private:
    MPU6050 _mpu6050;

};


#endif //MPU6050_INTERFACE