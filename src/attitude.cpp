#include "attitude.h"
#include "lsm9ds1_interface.h"
#include "mpu6050_interface.h"
#include "bno055_interface.h"
#include "joystick.h"
#include "user.h"
#include "maint.h"

#include <pico/float.h>

static const float DEGREES_TO_RADIANS = PI/180.f;
static const float RADIANS_TO_DEGREES = 180.0f/PI;
static const float CTRL_LOOP_PERIOD_S = 1.0f/CTRL_LOOP_FREQUENCY_HZ;
static const int   N_SAMPLE_SKIP = 500;

static int nSample = 0;
static int nGyroZ = 0;
static bool wasArmed = false;

float ATTITUDE_Roll0;
float ATTITUDE_Pitch0;

static ImuInterface* imu;

pt1_flt_tag ax_flt_tag;
pt1_flt_tag ay_flt_tag;
pt1_flt_tag az_flt_tag;
pt1_flt_tag gx_flt_tag;
pt1_flt_tag gy_flt_tag;
pt1_flt_tag gz_flt_tag;
pt1_flt_tag mx_flt_tag;
pt1_flt_tag my_flt_tag;
pt1_flt_tag mz_flt_tag;

static bool filter_on;

static float body_roll;
static float body_pitch;
static float body_yaw;


static float pt1f(float raw_k, float raw_km1, float filt_km1, float T_PTF1_S)
{
    return filt_km1 + (raw_k - filt_km1) * (1 - exp(-((1.0f/CTRL_LOOP_FREQUENCY_HZ)/T_PTF1_S)));
}


static void pt1f_init(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    ax_flt_tag = {ax, ax, ax, ax};
    ay_flt_tag = {ay, ay, ay, ay};
    az_flt_tag = {az, az, az, az};

    gx_flt_tag = {gx, gx, gx, gx};
    gy_flt_tag = {gy, gy, gy, gy};
    gz_flt_tag = {gz, gz, gz, gz};

    mx_flt_tag = {mx, mx, mx, mx};
    my_flt_tag = {my, my, my, my};
    mz_flt_tag = {mz, mz, mz, mz};
}


static void pt1f_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    ax_flt_tag.raw_k = ax;
    ay_flt_tag.raw_k = ay;
    az_flt_tag.raw_k = az;

    gx_flt_tag.raw_k = gx;
    gy_flt_tag.raw_k = gy;
    gz_flt_tag.raw_k = gz;

    mx_flt_tag.raw_k = mx;
    my_flt_tag.raw_k = my;
    mz_flt_tag.raw_k = mz;

    ax_flt_tag.filt_k = pt1f(ax_flt_tag.raw_k, ax_flt_tag.raw_km1, ax_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::ACCELEROMETER)][int(EUCLIDEAN_AXES::X)]));
    ay_flt_tag.filt_k = pt1f(ay_flt_tag.raw_k, ay_flt_tag.raw_km1, ay_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::ACCELEROMETER)][int(EUCLIDEAN_AXES::Y)]));
    az_flt_tag.filt_k = pt1f(az_flt_tag.raw_k, az_flt_tag.raw_km1, az_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::ACCELEROMETER)][int(EUCLIDEAN_AXES::Z)]));

    gx_flt_tag.filt_k = pt1f(gx_flt_tag.raw_k, gx_flt_tag.raw_km1, gx_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::GYROSCOPE)][int(EUCLIDEAN_AXES::X)]));
    gy_flt_tag.filt_k = pt1f(gy_flt_tag.raw_k, gy_flt_tag.raw_km1, gy_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::GYROSCOPE)][int(EUCLIDEAN_AXES::Y)]));
    gz_flt_tag.filt_k = pt1f(gz_flt_tag.raw_k, gz_flt_tag.raw_km1, gz_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::GYROSCOPE)][int(EUCLIDEAN_AXES::Z)]));

    mx_flt_tag.filt_k = pt1f(mx_flt_tag.raw_k, mx_flt_tag.raw_km1, mx_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::MAGNETOMETER)][int(EUCLIDEAN_AXES::X)]));
    my_flt_tag.filt_k = pt1f(my_flt_tag.raw_k, my_flt_tag.raw_km1, my_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::MAGNETOMETER)][int(EUCLIDEAN_AXES::Y)]));
    mz_flt_tag.filt_k = pt1f(mz_flt_tag.raw_k, mz_flt_tag.raw_km1, mz_flt_tag.filt_km1, *reinterpret_cast<float*>(&MAINT_Ptf1Parameters[int(SENSOR_SOURCE::MAGNETOMETER)][int(EUCLIDEAN_AXES::Z)]));

    ax_flt_tag.raw_km1 = ax_flt_tag.raw_k;
    ay_flt_tag.raw_km1 = ay_flt_tag.raw_k;
    az_flt_tag.raw_km1 = az_flt_tag.raw_k;

    gx_flt_tag.raw_km1 = gx_flt_tag.raw_k;
    gy_flt_tag.raw_km1 = gy_flt_tag.raw_k;
    gz_flt_tag.raw_km1 = gz_flt_tag.raw_k;

    mx_flt_tag.raw_km1 = mx_flt_tag.raw_k;
    my_flt_tag.raw_km1 = my_flt_tag.raw_k;
    mz_flt_tag.raw_km1 = mz_flt_tag.raw_k;

    ax_flt_tag.filt_km1 = ax_flt_tag.filt_k;
    ay_flt_tag.filt_km1 = ay_flt_tag.filt_k;
    az_flt_tag.filt_km1 = az_flt_tag.filt_k;

    gx_flt_tag.filt_km1 = gx_flt_tag.filt_k;
    gy_flt_tag.filt_km1 = gy_flt_tag.filt_k;
    gz_flt_tag.filt_km1 = gz_flt_tag.filt_k;

    mx_flt_tag.filt_km1 = mx_flt_tag.filt_k;
    my_flt_tag.filt_km1 = my_flt_tag.filt_k;
    mz_flt_tag.filt_km1 = mz_flt_tag.filt_k;
}


static void estimate_attitude()
{
    body_roll = atan2(ay_flt_tag.filt_k, az_flt_tag.filt_k) * RADIANS_TO_DEGREES;
    body_pitch = atan2(-ax_flt_tag.filt_k, sqrt(ay_flt_tag.filt_k * ay_flt_tag.filt_k + az_flt_tag.filt_k * az_flt_tag.filt_k)) * RADIANS_TO_DEGREES;

    body_roll = atan2(sin(body_roll * DEGREES_TO_RADIANS), cos(body_roll * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
    body_pitch = atan2(sin(body_pitch * DEGREES_TO_RADIANS), cos(body_pitch * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;

    float heading;

    if (!imu->haveMagneticField())
    {
        if (nSample < N_SAMPLE_SKIP)
        {
            nSample += 1;
        }
        else
        {
            /** Check if became disarmed **/
            if (!JOYSTICK_MotorsArmed && wasArmed)
            {
                body_yaw = 0;
                nSample = 0;
            }
            else
            {
                /** Integrate over gyro_z **/
                float dGz = -gz_flt_tag.filt_k;
                body_yaw += dGz * CTRL_LOOP_PERIOD_S;
                body_yaw = atan2(sin(body_yaw * DEGREES_TO_RADIANS), cos(body_yaw * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
            }
        }
    }
    else
    {
        if (my_flt_tag.filt_k == 0)
            heading = (mx_flt_tag.filt_k < 0) ? PI : 0;
        else
            heading = atan2(mx_flt_tag.filt_k, my_flt_tag.filt_k);

        /** Florence declination = 3.75 degrees **/
        heading -= 3.75f * DEGREES_TO_RADIANS;

        if (heading > PI)
            heading -= (2 * PI);
        else if (heading < -PI)
            heading += (2 * PI);

        body_yaw = heading * RADIANS_TO_DEGREES;
    }

    wasArmed = JOYSTICK_MotorsArmed;
}


bool ATTITUDE_Init()
{
    ax_flt_tag = {};
    ay_flt_tag = {};
    az_flt_tag = {};
    gx_flt_tag = {};
    gy_flt_tag = {};
    gz_flt_tag = {};
    mx_flt_tag = {};
    my_flt_tag = {};
    mz_flt_tag = {};

    body_roll = 0.0f;
    body_pitch = 0.0f;
    body_yaw = 0.0f;

    filter_on = false;

    ATTITUDE_Roll0 = 2.4375f;
    ATTITUDE_Pitch0 = 175.75f;

    switch (MAINT_ImuType)
    {
        case IMU_TYPE::LSM9DS1:
            imu = new LSM9DS1Interface();
            return imu->begin(i2c1, IMU_SDA_PIN, IMU_SCL_PIN);
        case IMU_TYPE::MPU6050:
            imu = new MPU6050Interface();
            return imu->begin(i2c1, IMU_SDA_PIN, IMU_SCL_PIN);
        case IMU_TYPE::BNO055:
            imu = new BNO055Interface();
            return imu->begin(i2c1, IMU_SDA_PIN, IMU_SCL_PIN);
        default:
            imu = new MPU6050Interface();
            return imu->begin(i2c1, IMU_SDA_PIN, IMU_SCL_PIN);
    }
}


void ATTITUDE_Handler()
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    
    imu->getAccel(&ax, &ay, &az);
    imu->getGyro(&gx, &gy, &gz);
    imu->getMagneticField(&mx, &my, &mz);

    if (imu->haveAbsoluteOrientation())
    {
        pt1f_init(ax, ay, az, gx, gy, gz, mx, my, mz);
        imu->getAbsoluteOrientation(&body_roll, &body_pitch, &body_yaw);
        body_roll = atan2(sin(body_roll * DEGREES_TO_RADIANS), cos(body_roll * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
        body_pitch = atan2(sin(body_pitch * DEGREES_TO_RADIANS), cos(body_pitch * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
    }
    else
    {
        if (filter_on == false)
        {
            pt1f_init(ax, ay, az, gx, gy, gz, mx, my, mz);
            filter_on = true;
        }
        else
        {
            pt1f_update(ax, ay, az, gx, gy, gz, mx, my, mz);
        }

        estimate_attitude();
    }
}


void ATTITUDE_Calibrate()
{
    ATTITUDE_Handler();

    if (ATTITUDE_RelRoll() != 0.0f)
    {
        ATTITUDE_Roll0 = body_roll;
    }
    
    if (ATTITUDE_RelPitch() != 0.0f)
    {
        ATTITUDE_Pitch0 = body_pitch;
    }
}


float ATTITUDE_RelRoll()
{
    return atan2(sin((body_roll - ATTITUDE_Roll0) * DEGREES_TO_RADIANS), cos((body_roll - ATTITUDE_Roll0) * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
}


float ATTITUDE_RelPitch()
{
    return atan2(sin((body_pitch - ATTITUDE_Pitch0) * DEGREES_TO_RADIANS), cos((body_pitch - ATTITUDE_Pitch0) * DEGREES_TO_RADIANS)) * RADIANS_TO_DEGREES;
}


float ATTITUDE_AbsYaw()
{
    return body_yaw;
}