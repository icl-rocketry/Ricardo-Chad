#pragma once

#include <libriccore/riccoresystem.h>
#include <librrc/Remote/nrcremoteservo.h>


#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include <libriccore/networkinterfaces/can/canbus.h>
#include "GNC/GNCWatchDog.h"
#include "GNC/nrcremoteprop.h"

#include "Commands/commands.h"

#include "SiC43x.h"
#include "GNC/NTRIPConnector.h"

class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        void sendCommand();

        SiC43x Buck;

        CanBus<SYSTEM_FLAG> canbus;
        
        NTRIPConnector ntrip;
    private:
        
        NRCRemoteProp Motor1;
        NRCRemoteProp Motor2;
        GNCWatchDog clifford;//🐶
    protected: 

        
            bool fullbore_called = false;
            bool shutdown_called = false;

    
};