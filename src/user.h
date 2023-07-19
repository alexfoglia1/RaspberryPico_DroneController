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

#define CHANNEL_ROLL     15
#define CHANNEL_PITCH    14
#define CHANNEL_THROTTLE 13
#define CHANNEL_ARMED    12

#define MOTOR_1 8
#define MOTOR_2 9
#define MOTOR_3 10
#define MOTOR_4 11

void InitBoard();

#endif //USER_H