#ifndef BNO055_INTERFACE
#define BNO055_INTERFACE

#include "imu_interface.h"
#include "bno055.h"

class BNO055Interface : public ImuInterface
{

public:

    BNO055Interface();
    
    bool begin(i2c_inst_t* i2c_channel, int sdaPin, int sclPin) override;
    bool haveMagneticField() override;
    bool haveAbsoluteOrientation() override;
    void getGyro(float* x, float* y, float* z) override;
    void getAccel(float* x, float* y, float* z) override;
    void getMagneticField(float* x, float* y, float* z) override;
    void getAbsoluteOrientation(float* roll, float* pitch, float* yaw) override;

private:
    equipment_handlers::BNO055_IMU _bno055;
    float _roll0;
    float _pitch0;

};


#endif //BNO055_INTERFACE