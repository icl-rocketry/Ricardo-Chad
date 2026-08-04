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

    static constexpr uint8_t sclPin = 10;
    static constexpr uint8_t sdaPin = 9;
    static constexpr uint8_t resetPin = 7;
    static constexpr uint8_t dcPin = 6;    
    static constexpr uint8_t csPin = 5;
};


