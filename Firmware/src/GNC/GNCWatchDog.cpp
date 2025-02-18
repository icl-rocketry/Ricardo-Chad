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

#include "States/idle.h"

#include <librrc/Interface/rocketcomponent.h>

GNCWatchDog::GNCWatchDog(RnpNetworkManager &networkmanager):
m_networkmanager(networkmanager),
PIDcheck(1, GeneralConfig::pickleAddr, static_cast<uint8_t>(Services::ID::PID), static_cast<uint8_t>(Services::ID::PID), m_networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
PicklePoller(300, &PIDcheck)
{};
void GNCWatchDog::watchDogSetup(){

    PicklePoller.setup();
}

void GNCWatchDog::watchDogUpdate(){
    try {
       PicklePoller.update();
    } catch (const std::exception &e) {
        
        
    }
}

