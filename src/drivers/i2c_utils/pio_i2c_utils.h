#ifndef PIO_I2C_UTILS_H
#define PIO_I2C_UTILS_H

#include <stdint.h>
extern "C"
{
    #include "pio_i2c.h"
}

int pioI2cWriteByteToRegister(PIO pio, uint8_t addr, uint8_t reg, uint8_t byte);
int pioI2cWriteWordToRegister(PIO pio, uint8_t addr, uint8_t reg, int16_t word, bool littleEndian=true);
int pioI2cWriteDataToRegister(PIO pio, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len);

int16_t pioI2cReadWordFromRegister(PIO pio, uint8_t addr, uint8_t reg, bool littleEndian=true);
uint8_t pioI2cReadByteFromRegister(PIO pio, uint8_t addr, uint8_t reg);
void pioI2cReadDataFromRegister(PIO pio, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len);

#endif