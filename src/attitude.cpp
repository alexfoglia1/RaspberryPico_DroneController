#include "attitude.h"
#include "LSM9DS1.h"
#include "user.h"
#include "maint.h"

#include <stdio.h>
#include <pico/float.h>

static const float DEGREES_TO_RADIANS = PI/180.f;
static const float RADIANS_TO_DEGREES = 180.0f/PI;
static const float CTRL_LOOP_PERIOD = 1.0f/CTRL_LOOP_FREQUENCY_HZ;

static LSM9DS1 imu;

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

float ATTITUDE_Roll;
float ATTITUDE_Pitch;
float ATTITUDE_Yaw;

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
    ATTITUDE_Roll = atan2(ay_flt_tag.filt_k, az_flt_tag.filt_k) * RADIANS_TO_DEGREES;
    ATTITUDE_Pitch = atan2(-ax_flt_tag.filt_k, sqrt(ay_flt_tag.filt_k * ay_flt_tag.filt_k + az_flt_tag.filt_k * az_flt_tag.filt_k)) * RADIANS_TO_DEGREES;

    float heading;
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

    ATTITUDE_Yaw = heading * RADIANS_TO_DEGREES;
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

    ATTITUDE_Roll = 0.0f;
    ATTITUDE_Pitch = 0.0f;
    ATTITUDE_Yaw = 0.0f;

    filter_on = false;

    return imu.begin();
}


void ATTITUDE_Handler()
{
    if (imu.accelAvailable()) imu.readAccel();
    if (imu.gyroAvailable()) imu.readGyro();
    if (imu.magAvailable()) imu.readMag();

    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    float gx = imu.calcGyro(imu.gx);
    float gy = imu.calcGyro(imu.gy);
    float gz = imu.calcGyro(imu.gz);

    float mx = -1 * imu.calcMag(imu.mx); // X/Y inverted
    float my = -1 * imu.calcMag(imu.my);
    float mz = imu.calcMag(imu.mz);

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