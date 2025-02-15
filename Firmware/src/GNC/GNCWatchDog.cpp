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

GNCWatchDog::GNCWatchDog();
PicklePoller(300, &OxTankPTap)
{};
void GNCWatchDog::watchDogSetup(){

    PicklePoller.setup();
}

void GNCWatchDog::watchDogUpdate(){
    try {
       PicklePoller.update();
    } catch (const std::runtime_error("Sensor with ID: " + std::to_string(_networksensor->getID()) + " not responding to poll request");) {
        std::cout << "Caught an exception: " << e.what() << std::endl;
    }
}

