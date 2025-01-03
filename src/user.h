#ifndef USER_H
#define USER_H

#define I2C_FREQUENCY_HZ 400000
#define JS_TIMEOUT_MILLIS 25 // 1 expected pwm period + 5 millis
#define MAINT_TIMEOUT_MILLIS 1000
#define MILLISECONDS_TO_MICROSECONDS 1000UL
#define SECONDS_TO_MICROSECONDS 1000000UL

#define CTRL_LOOP_FREQUENCY_HZ 200

#ifdef __JOYMODE_2__
#define CHANNEL_THROTTLE_GPIO 13 // RADIO CHANNEL 4
#define CHANNEL_PITCH_GPIO    12 // RADIO CHANNEL 2
#define CHANNEL_ROLL_GPIO     11 // RADIO CHANNEL 1
#define CHANNEL_ARMED_GPIO    10 // RADIO CHANNEL 5
#else
#define CHANNEL_ROLL_GPIO     13 // RADIO CHANNEL 4
#define CHANNEL_THROTTLE_GPIO 12 // RADIO CHANNEL 2
#define CHANNEL_PITCH_GPIO    11 // RADIO CHANNEL 1
#define CHANNEL_ARMED_GPIO    10 // RADIO CHANNEL 5
#endif

#define MOTOR_1_GPIO 8
#define MOTOR_2_GPIO 7
#define MOTOR_3_GPIO 6
#define MOTOR_4_GPIO 9

#define IMU_SDA_PIN 2
#define IMU_SCL_PIN 3

#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define PROBE_GPIO_CPU0 20
#define PROBE_GPIO_CPU1 21

#define MOTOR_ARMED_THRESHOLD 80 //TODO: Rimettere 250

#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint8_t major_v;
    uint8_t minor_v;
    uint8_t stage_v;
    uint8_t rel_type;
} SW_VER_TAG;

enum class REL_TYPE_TAG : uint8_t
{
    BETA = 0x00,
    RELEASE = 0x01
};

extern const uint8_t MAJOR_V;
extern const uint8_t MINOR_V;
extern const uint8_t STAGE_V;
extern const REL_TYPE_TAG REL_TYPE;

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