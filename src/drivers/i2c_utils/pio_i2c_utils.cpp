extern "C"
{
    #include "pio_i2c_utils.h"
}

#include <string.h>


int pioI2cWriteDataToRegister(PIO pio, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    uint8_t* buf = new uint8_t[data_len + 1];
    buf[0] = static_cast<uint8_t>(reg);

    memcpy(&buf[1], data, data_len);
    
    int write_len = pio_i2c_write_blocking(pio, 0, addr, buf, data_len + 1);

    if (write_len != data_len)
    {
        delete[] buf;

        return write_len;
    }
    else
    {
        return PICO_ERROR_GENERIC;
    }
}


int pioI2cWriteByteToRegister(PIO pio, uint8_t addr, uint8_t reg, uint8_t byte)
{
    return pioI2cWriteDataToRegister(pio, addr, reg, &byte, 1);
}


int i2cWriteWordToRegister(PIO pio, uint8_t addr, uint8_t reg, int16_t word, bool littleEndian)
{
    uint8_t* data = reinterpret_cast<uint8_t*>(&word);
    if (littleEndian)
    {
        uint8_t tmp = data[0];
        data[0] = data[1];
        data[1] = tmp;
    }

    return pioI2cWriteDataToRegister(pio, addr, reg, data, 2);
}


int16_t pioI2cReadWordFromRegister(PIO pio, uint8_t addr, uint8_t reg, bool littleEndian)
{
    uint16_t val;

    uint8_t u8reg = static_cast<uint8_t>(reg);
    pio_i2c_read_blocking(pio, 0, addr, reinterpret_cast<uint8_t*>(&val), 2);

    if (littleEndian)
    {
        uint16_t LSB = (val & 0xFF00) >> 8;
        uint16_t MSB = (val & 0x00FF);
    
        return ((MSB << 8) | LSB);
    }
    else
    {
        return val;
    }
}

uint8_t pioI2cReadByteFromRegister(PIO pio, uint8_t addr, uint8_t reg)
{
    uint8_t val;

    uint8_t u8reg = static_cast<uint8_t>(reg);
    pio_i2c_read_blocking(pio, 0, addr, &val, 1);

    return val;
}


void pioI2cReadDataFromRegister(PIO pio, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    uint8_t u8reg = static_cast<uint8_t>(reg);
    pio_i2c_read_blocking(pio, 0, addr, data, data_len);
}

