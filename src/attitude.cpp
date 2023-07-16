#include "attitude.h"
#include "LSM9DS1.h"
#include "user.h"
#include <stdio.h>
#include <pico/float.h>

static LSM9DS1 imu;

static float ax_raw_km1;
static float ay_raw_km1;
static float az_raw_km1;
static float ax_flt_km1;
static float ay_flt_km1;
static float az_flt_km1;
static float mx_raw_km1;
static float my_raw_km1;
static float mz_raw_km1;
static float mx_flt_km1;
static float my_flt_km1;
static float mz_flt_km1;
static bool filter_on;

float ATTITUDE_Roll;
float ATTITUDE_Pitch;
float ATTITUDE_Yaw;

static float pt1f(float raw_k, float raw_km1, float filt_km1, float T_PTF1_S)
{
    return filt_km1 + (raw_k - filt_km1) * (1 - exp(-((1.0f/CTRL_LOOP_FREQUENCY_HZ)/T_PTF1_S)));
}

bool ATTITUDE_Init(void)
{
    ax_raw_km1 = 0.0f;
    ay_raw_km1 = 0.0f;
    az_raw_km1 = 0.0f;
    ax_flt_km1 = 0.0f;
    ay_flt_km1 = 0.0f;
    az_flt_km1 = 0.0f;
    mx_raw_km1 = 0.0f;
    my_raw_km1 = 0.0f;
    mz_raw_km1 = 0.0f;
    mx_flt_km1 = 0.0f;
    my_flt_km1 = 0.0f;
    mz_flt_km1 = 0.0f;

    ATTITUDE_Roll = 0.0f;
    ATTITUDE_Pitch = 0.0f;
    ATTITUDE_Yaw = 0.0f;

    filter_on = false;

    return imu.begin();
}


void ATTITUDE_Handler(void)
{
    imu.readAccel();
    imu.readGyro();
    imu.readMag();

    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    float mx = -1 * imu.calcMag(imu.my); // X/Y inverted
    float my = -1 * imu.calcMag(imu.mx);
    float mz = imu.calcMag(imu.mz);

    if (filter_on == false)
    {
        ax_raw_km1 = ax;
        ay_raw_km1 = ay;
        az_raw_km1 = az;

        ax_flt_km1 = ax;
        ay_flt_km1 = ay;
        az_flt_km1 = az;

        mx_raw_km1 = mx;
        my_raw_km1 = my;
        mz_raw_km1 = mz;

        mx_flt_km1 = mx;
        my_flt_km1 = my;
        mz_flt_km1 = mz;

        filter_on = true;
    }
    else
    {
        ax_raw_km1 = ax;
        ay_raw_km1 = ay;
        az_raw_km1 = az;
        mx_raw_km1 = mx;
        my_raw_km1 = my;
        mz_raw_km1 = mz;

        ax = pt1f(ax, ax_raw_km1, ax_flt_km1, 0.2f);
        ay = pt1f(ay, ay_raw_km1, ay_flt_km1, 0.2f);
        az = pt1f(az, az_raw_km1, az_flt_km1, 0.2f);
        mx = pt1f(mx, mx_raw_km1, mx_flt_km1, 0.4f);
        my = pt1f(my, my_raw_km1, my_flt_km1, 0.4f);
        mz = pt1f(mz, mz_raw_km1, mz_flt_km1, 0.4f);

        ax_flt_km1 = ax;
        ay_flt_km1 = ay;
        az_flt_km1 = az;
        mx_flt_km1 = mx;
        my_flt_km1 = my;
        mz_flt_km1 = mz;
    }

    ATTITUDE_Roll = atan2(ay, az) * 180.0f / PI;
    ATTITUDE_Pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    float heading;
    if (my == 0)
    {
        heading = (mx < 0) ? PI : 0;
    }
    else
    {
        heading = atan2(mx, my);
    }

    heading -= 3.75f * PI / 180;

    if (heading > PI)
    {
        heading -= (2 * PI);
    }
    else if (heading < -PI)
    {
        heading += (2 * PI);
    }

    ATTITUDE_Yaw = heading  * 180.0f / PI;
}