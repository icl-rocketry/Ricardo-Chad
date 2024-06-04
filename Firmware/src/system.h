#pragma once

#include <libriccore/riccoresystem.h>
#include <librrc/nrcremoteservo.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include <libriccore/networkinterfaces/can/canbus.h>

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

        NRCRemoteServo Servo1;
        NRCRemoteServo Servo2;
        NRCHelikopterHelikopter HelikopterHelikopter; //ya this does nothing yet
    //everything below this is done by arlo, idk if it works yet😔
    protected: 

        enum class GNCState : uint8_t
            {
                Default = 0,
                Arm = 1,
                Hover = 2,
                Abort = 3,
               
            };

            bool fullbore_called = false;
            bool shutdown_called = false;

            EngineState currentEngineState = EngineState::Default;

        

};