#pragma once

#include <libriccore/riccoresystem.h>
#include <librrc/Remote/nrcremoteservo.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include <libriccore/networkinterfaces/can/canbus.h>
#include <librrc/HAL/localpwm.h>

#include "Commands/commands.h"

#include "SiC43x.h"
#include <Wire.h>
#include <Arduino.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <SPI.h>

class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        SiC43x Buck;

        CanBus<SYSTEM_FLAG> canbus;
        
        
    private:

        LocalPWM m_servo0_pwm;
        LocalPWM m_servo1_pwm;

        NRCRemoteServo<LocalPWM> m_servo0;
        NRCRemoteServo<LocalPWM> m_servo1;
        
        ADC_VRailMonitor pot0;
        ADC_VRailMonitor pot1;
        TwoWire i2cBus;
        U8G2_SSD1322_ZJY_256X64_F_4W_SW_SPI display;

        int PotLowerVThreshhold = 100;
        int PotUpperVThreshhold = 3100;
        uint16_t PotSampleRate = 20; // Sample the potentiometer at 10hz by default

        float Pot0OutputV = 0;
        int Pot0Percentage = 0;
        int Pot0PercentageOld = 0;
        

};