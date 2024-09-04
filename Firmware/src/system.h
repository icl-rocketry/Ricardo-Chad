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

        NRCRemoteServo<LocalPWM> Servo0;
        NRCRemoteServo<LocalPWM> Servo1;
        

};