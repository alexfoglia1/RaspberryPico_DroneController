#ifndef USER_H
#define USER_H

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQUENCY_HZ 400000

#define JS_TIMEOUT_S 2
#define MAINT_TIMEOUT_S 1
#define SECONDS_TO_MICROSECONDS 1000000UL

#define CTRL_LOOP_FREQUENCY_HZ 500

#define CHANNEL_ROLL     15 // RADIO CHANNEL 1
#define CHANNEL_PITCH    14 // RADIO CHANNEL 2
#define CHANNEL_THROTTLE 13 // RADIO CHANNEL 3
#define CHANNEL_ARMED    12 // RADIO CHANNEL 5

#define MOTOR_1 8
#define MOTOR_2 9
#define MOTOR_3 10
#define MOTOR_4 11

#define PROBE 16

#define MOTOR_ARMED_THRESHOLD 100

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