#ifndef USER_H
#define USER_H

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQUENCY_HZ 400000

#define CTRL_LOOP_FREQUENCY_HZ 500

#define CHANNEL_ROLL     15
#define CHANNEL_PITCH    14
#define CHANNEL_THROTTLE 13
#define CHANNEL_ARMED    12

#define MOTOR_1 4
#define MOTOR_2 5
#define MOTOR_3 6
#define MOTOR_4 7

void InitBoard();

#endif //USER_H