#ifndef USER_H
#define USER_H

#define I2C_FREQUENCY_HZ 400000

#define JS_TIMEOUT_S 2
#define MAINT_TIMEOUT_S 1
#define SECONDS_TO_MICROSECONDS 1000000UL

#define CTRL_LOOP_FREQUENCY_HZ 500

#define CHANNEL_ROLL_GPIO     13 // RADIO CHANNEL 1
#define CHANNEL_PITCH_GPIO    12 // RADIO CHANNEL 2
#define CHANNEL_THROTTLE_GPIO 11 // RADIO CHANNEL 3
#define CHANNEL_ARMED_GPIO    10 // RADIO CHANNEL 5

#define MOTOR_1_GPIO 8
#define MOTOR_2_GPIO 7
#define MOTOR_3_GPIO 6
#define MOTOR_4_GPIO 9

#define BNO055_SDA_PIN 2
#define BNO055_SCL_PIN 3
#define LSM9DS1_SDA_PIN 4
#define LSM9DS1_SCL_PIN 5
#define MPU6050_SDA_PIN 18
#define MPU6050_SCL_PIN 19

#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define PROBE_GPIO 16

#define MOTOR_ARMED_THRESHOLD 200

#include <stdint.h>

typedef struct
{
    uint8_t major_v;
    uint8_t minor_v;
    uint8_t stage_v;
    uint8_t rel_type;
} SW_VER_TAG;

enum class REL_TYPE_TAG
{
    BETA = 0x00,
    RELEASE = 0x01
};

#define MAJOR_V  1
#define MINOR_V  0
#define STAGE_V  1
#define REL_TYPE REL_TYPE_TAG::BETA

inline float saturate(float val, float min, float max)
{
    return (val < min) ? min :
           (val > max) ? max :
           val;
}

inline float to_range(float in, float in_min, float in_max, float out_min, float out_max)
{
    in = saturate(in, in_min, in_max);
    float in_percentage = (in - in_min) / (in_max - in_min);
    
    return out_min + (in_percentage) * (out_max - out_min);
}

void InitBoard();

#endif //USER_H