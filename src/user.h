#ifndef USER_H
#define USER_H

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQUENCY_HZ 400000

#define CTRL_LOOP_FREQUENCY_HZ 500

void InitBoard(void);

#endif //USER_H