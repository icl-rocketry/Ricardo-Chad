#pragma once

#include <stdint.h>
#include <type_traits>

enum class SYSTEM_FLAG:uint32_t{
    //state flags
    STATE_IDLE = (1 << 0), 
    //flags
    DEBUG = (1 << 7),
    //critical messages 
    ERROR_SERIAL = (1 << 10),
    ERROR_CAN = (1 << 11),
    //Hardware errors
    ERROR_PGOOD = (1 << 12), //Power good flag is in the wrong state
    ERROR_VBOUNDS = (1 << 13) //Buck output voltage is out of bounds
    
};

using system_flag_t = uint32_t;

