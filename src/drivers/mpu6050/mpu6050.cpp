#include "mpu6050.h"
#include "i2c_utils.h"

#include <string.h>
#include <limits>
#include <hardware/i2c.h>
#include <stdio.h>

MPU6050::MPU6050()
{
    _accelResolution = MPU6050::AccelResolution::ACCEL_2G;
    _gyroResolution = MPU6050::GyroResolution::GYRO_250_DPS;

    _gyroBiasX = 0;
    _gyroBiasY = 0;
    _gyroBiasZ = 0;
}

bool MPU6050::init()
{
    /** Set sample rate 8 kHz **/
    bool res = (i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::SMPLRT_DIV), 0) != PICO_ERROR_GENERIC);

    /** Reset all sensors **/
    res &= (i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::PWR_MGMT_1), 0) != PICO_ERROR_GENERIC);

    res &= (i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::PWR_MGMT_1), 1) != PICO_ERROR_GENERIC);

    /** Reset configuration **/
    res &= (i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::CONFIG), 0)  != PICO_ERROR_GENERIC);
    res &= (setGyroResolution(_gyroResolution) != PICO_ERROR_GENERIC);
    res &= (setAccelResolution(_accelResolution) != PICO_ERROR_GENERIC);

    /** Enable interrupts **/
    res &= (i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::INT_ENABLE), 0) != PICO_ERROR_GENERIC);

    calcGyroBias();

    return res;
}


int MPU6050::setGyroResolution(MPU6050::GyroResolution gyroResolution)
{
    _gyroResolution = gyroResolution;

    uint8_t byte = static_cast<uint8_t>(_gyroResolution);
    return i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::GYRO_CONFIG), byte);
}


int MPU6050::setAccelResolution(MPU6050::AccelResolution accelResolution)
{
    _accelResolution = accelResolution;

    uint8_t byte = static_cast<uint8_t>(_accelResolution);
    return i2cWriteByteToRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::ACCEL_CONFIG), byte);
}


void MPU6050::readGyro(float* gx, float* gy, float* gz)
{
    int16_t rawGyroX = i2cReadWordFromRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::GYRO_XOUT_H));
    int16_t rawGyroY = i2cReadWordFromRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::GYRO_YOUT_H));
    int16_t rawGyroZ = i2cReadWordFromRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::GYRO_ZOUT_H));

    *gx = toDps(rawGyroX) - _gyroBiasX;
    *gy = toDps(rawGyroY) - _gyroBiasY;
    *gz = -toDps(rawGyroZ) - _gyroBiasZ;
}


void MPU6050::readAccel(float* ax, float* ay, float* az)
{
    int16_t rawAccelX = i2cReadWordFromRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::ACCEL_XOUT_H));
    int16_t rawAccelY = i2cReadWordFromRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::ACCEL_YOUT_H));
    int16_t rawAccelZ = i2cReadWordFromRegister(i2c1, MPU6050::I2C_ADDR, static_cast<uint8_t>(MPU6050::Register::ACCEL_ZOUT_H));

    *ax = toG(rawAccelX);
    *ay = toG(rawAccelY);
    *az = -toG(rawAccelZ);
}


float MPU6050::toDps(int16_t gyroRaw)
{
    float gyroResolution = gyroResolutionValue(_gyroResolution);
    return (static_cast<float>(gyroRaw) / static_cast<float>(std::numeric_limits<int16_t>::max())) * gyroResolution;
}


float MPU6050::toG(int16_t accelRaw)
{
    float accelResolution = accelResolutionValue(_accelResolution);
    return (static_cast<float>(accelRaw) / static_cast<float>(std::numeric_limits<int16_t>::max())) * accelResolution;
}


void MPU6050::calcGyroBias()
{
    const int N = 1000;

    _gyroBiasX = 0;
    _gyroBiasY = 0;
    _gyroBiasZ = 0;

    float gyroBiasX = 0;
    float gyroBiasY = 0;
    float gyroBiasZ = 0;
    float _gx = 0;
    float _gy = 0;
    float _gz = 0;
    for (int i = 0; i < N; i++)
    {
        readGyro(&_gx, &_gy, &_gz);

        gyroBiasX += _gx;
        gyroBiasY += _gy;
        gyroBiasZ += _gz;

        sleep_us(1000);
    }

    _gyroBiasX = gyroBiasX / static_cast<float>(N);
    _gyroBiasY = gyroBiasY / static_cast<float>(N);
    _gyroBiasZ = gyroBiasZ / static_cast<float>(N);
}


float MPU6050::gyroResolutionValue(MPU6050::GyroResolution res)
{
    float _ret = 0.0f;
    switch (res)
    {
        case MPU6050::GyroResolution::GYRO_250_DPS:  _ret = 250.f;  break;
        case MPU6050::GyroResolution::GYRO_500_DPS:  _ret = 500.f;  break;
        case MPU6050::GyroResolution::GYRO_1000_DPS: _ret = 1000.f; break;
        case MPU6050::GyroResolution::GYRO_2000_DPS: _ret = 2000.f; break;
    }

    return _ret;
}


float MPU6050::accelResolutionValue(MPU6050::AccelResolution res)
{
    float _ret = 0.0f;
    switch (res)
    {
        case MPU6050::AccelResolution::ACCEL_2G:  _ret = 2.f;  break;
        case MPU6050::AccelResolution::ACCEL_4G:  _ret = 4.f;  break;
        case MPU6050::AccelResolution::ACCEL_8G:  _ret = 8.f;  break;
        case MPU6050::AccelResolution::ACCEL_16G: _ret = 16.f; break;
    }

    return _ret;
}