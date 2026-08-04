/*
**********************
* PINS               *
**********************
 */
#pragma once
#include <stdint.h>

namespace PinMap{
    
    static constexpr uint8_t TxCan = 33;
    static constexpr uint8_t RxCan = 34;

    static constexpr uint8_t ServoPWM0 = 36;
    static constexpr uint8_t ServoPWM1 = 37;

    static constexpr uint8_t BuckEN = 38;
    static constexpr uint8_t BuckPGOOD = 47;
    static constexpr uint8_t BuckOutputV = 4;

    static constexpr uint8_t Pot0Control = 6;
    static constexpr uint8_t Pot1Control = 7;
    static constexpr uint8_t Pot2Control = 9;
    static constexpr uint8_t Pot3Control = 10;
};


