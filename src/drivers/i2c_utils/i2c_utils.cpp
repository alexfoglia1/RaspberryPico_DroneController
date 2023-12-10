#include "i2c_utils.h"

#include <string.h>

int i2cWriteDataToRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    uint8_t* buf = new uint8_t[data_len + 1];
    buf[0] = static_cast<uint8_t>(reg);

    memcpy(&buf[1], data, data_len);
    
    return i2c_write_blocking(i2c_bus, addr, buf, 2, false);
}


int i2cWriteDataToRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t byte)
{
    return i2cWriteDataToRegister(i2c_bus, addr, reg, &byte, 1);
}


int i2cWriteWordToRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, int16_t word, bool littleEndian)
{
    uint8_t* data = reinterpret_cast<uint8_t*>(&word);
    if (littleEndian)
    {
        uint8_t tmp = data[0];
        data[0] = data[1];
        data[1] = tmp;
    }

    return i2cWriteDataToRegister(i2c_bus, addr, reg, data, 2);
}


int16_t i2cReadWordFromRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, bool littleEndian)
{
    uint16_t val;

    uint8_t u8reg = static_cast<uint8_t>(reg);
    i2c_write_blocking(i2c_bus, addr, &u8reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_bus, addr, reinterpret_cast<uint8_t*>(&val), 2, false);

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

uint8_t i2cReadByteFromRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg)
{
    uint8_t val;

    uint8_t u8reg = static_cast<uint8_t>(reg);
    i2c_write_blocking(i2c_bus, addr, &u8reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_bus, addr, &val, 1, false);

    return val;
}


void i2cReadDataFromRegister(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    uint8_t u8reg = static_cast<uint8_t>(reg);
    i2c_write_blocking(i2c_bus, addr, &u8reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_bus, addr, data, data_len, false);
}

