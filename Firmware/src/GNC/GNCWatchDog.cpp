#include "GNCWatchDog.h"

#include <memory>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

#include "Commands/commands.h"
#include "Config/pinmap_config.h"

#include "States/idle.h"

#include <librrc/Interface/rocketcomponent.h>

GNCWatchDog::GNCWatchDog(RnpNetworkManager &networkmanager,NRCRemoteProp &motor1,NRCRemoteProp &motor2):
m_networkmanager(networkmanager),
PIDcheck(1, GeneralConfig::pickleAddr, static_cast<uint8_t>(Services::ID::PID), static_cast<uint8_t>(Services::ID::PID), m_networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
PicklePoller(300, &PIDcheck),
Motor1(motor1),
Motor2(motor2)
{};


void GNCWatchDog::watchDogSetup(){

    PicklePoller.setup(),
    oldTime = 0;

}

void GNCWatchDog::watchDogUpdate(){
    currentTime = millis();
    timeElapsed = currentTime - oldTime;

    if(timeElapsed > timeInterval){
        try {

        PicklePoller.update();
        
        } catch (const std::exception &e) {//catch if no response
        Motor1.turnOff(); 
        Motor2.turnOff(); 
        }
        if (requestTime > 15){ // if no update for 15 seconds kill motors

            Motor1.turnOff(); 
            Motor2.turnOff(); 
        }
        oldTime = currentTime; 
}
}

