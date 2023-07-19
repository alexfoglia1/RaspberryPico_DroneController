#ifndef CBIT_H
#define CBIT_H

#include <stdint.h>

typedef union
{
    struct
    {
       uint32_t 
    } Bits;
    uint8_t Bytes[4];
    uint32_t Dword;

} CBIT_TAG;



#endif //CBIT_H
