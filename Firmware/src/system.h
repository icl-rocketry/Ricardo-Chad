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
        ADC_VRailMonitor pot2;
        ADC_VRailMonitor pot3;

        bool sendInfoPacket = false;
        int PotLowerVThreshhold = 100;
        int PotUpperVThreshhold = 3100;
        uint16_t PotSampleRate = 20; // Sample the potentiometer at 10hz by default

        float Pot0OutputV = 0;
        float Pot0OutputVOld = 0;
        float Pot0PercentageRaw = 0;
        int Pot0Percentage = 0;
        int Pot0PercentageOld = 0;

        float Pot1OutputV = 0;
        float Pot1OutputVOld = 0;
        float Pot1PercentageRaw = 0;
        int Pot1Percentage = 0;
        int Pot1PercentageOld = 0;

        float Pot2OutputV = 0;
        float Pot2OutputVOld = 0;
        float Pot2PercentageRaw = 0;
        int Pot2Percentage = 0;
        int Pot2PercentageOld = 0;

        float Pot3OutputV = 0;
        float Pot3OutputVOld = 0;
        float Pot3PercentageRaw = 0;
        int Pot3Percentage = 0;
        int Pot3PercentageOld = 0;
        
        double alpha = 0.6;

};