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
class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        SiC43x Buck;

        CanBus<SYSTEM_FLAG> canbus;
    private:
        
        NRCRemoteProp Motor1;
        NRCRemoteProp Motor2;

    protected: 

        
            bool fullbore_called = false;
            bool shutdown_called = false;

    
};