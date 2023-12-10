#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include <stdint.h>
#include <hardware/i2c.h>

int i2cWriteDataToRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t byte);
int i2cWriteWordToRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, int16_t word, bool littleEndian=true);
int i2cWriteDataToRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len);

int16_t i2cReadWordFromRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, bool littleEndian=true);
uint8_t i2cReadByteFromRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg);
void i2cReadDataFromRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len);

#endif