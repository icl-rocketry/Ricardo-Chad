#pragma once

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>
#include <libriccore/platform/esp32/ADC.h>
#include <librrc/Interface/networksensor.h>
#include <librrc/Helpers/sensorpoller.h>


#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

#include "nrcremoteprop.h"

#include "SiC43x.h"



#include "Commands/commands.h"

class GNCWatchDog 
{
public:
    GNCWatchDog(RnpNetworkManager& networkmanager,NRCRemoteProp& motor1,NRCRemoteProp& motor2);

    RnpNetworkManager& m_networkmanager;
    NetworkSensor PIDcheck;
    SensorPoller PicklePoller;
    NRCRemoteProp& Motor1;
    NRCRemoteProp& Motor2;

    void setup();
    void update();


private:

    
    double requestTime;
    double timeInterval = 1000;

    double timeElapsed; 
    //equal to one to force pickle poll
    double currentTime;
    double oldTime;


};
