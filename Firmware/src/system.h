#pragma once

#include <libriccore/riccoresystem.h>
#include <librrc/Remote/nrcremoteservo.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include <libriccore/networkinterfaces/can/canbus.h>
#include <libriccore/networkinterfaces/serial/streamserial.h>
#include <librrc/HAL/localpwm.h>

#include "TMC2209/tmc2209.h"

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
        StreamSerial<SYSTEM_FLAG> abus;

        //TMC2209 tmc;

    private:
        static constexpr bool m_buck_idle_powerdown { true };

        LocalPWM m_servo0_pwm;
        LocalPWM m_servo1_pwm;

        NRCRemoteServo<LocalPWM> m_servo0;
        NRCRemoteServo<LocalPWM> m_servo1;
};