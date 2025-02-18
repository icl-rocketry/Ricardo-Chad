#pragma once

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>
#include <libriccore/platform/esp32/ADC.h>
#include <librrc/Interface/networksensor.h>
#include <librrc/Helpers/sensorpoller.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"

#include <librrc/Remote/nrcremoteservo.h>
#include "GNC/nrcremoteprop.h"


#include "SiC43x.h"


#include "Commands/commands.h"

class GNCWatchDog 
{
public:
    GNCWatchDog(RnpNetworkManager &m_networkmanager,NRCRemoteProp &Motor1,NRCRemoteProp &Motor2);

    void watchDogSetup();
    void watchDogUpdate();

    NetworkSensor PIDcheck;
    SensorPoller PicklePoller;

private:

    RnpNetworkManager &m_networkmanager;
    
    NRCRemoteProp Motor1;
    NRCRemoteProp Motor2;
    double requestTime;
    double timeInterval = 0.1;

    double timeElapsed; 
    //equal to one to force pickle poll
    double currentTime;
    double oldTime;


};
