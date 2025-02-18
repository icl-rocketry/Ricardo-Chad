#include "system.h"

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
#include "GNC/GNCWatchDog.h"

System::System():
RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
Buck(systemstatus, PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 1500, 470),
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
Motor1(PinMap::ServoPWM0, 0, networkmanager),
Motor2(PinMap::ServoPWM1, 1, networkmanager)
// gnc(networkmanager,PinMap::ServoPWM1, 0, PinMap::ServoPWM2, 1, networkmanager.getAddress())
// GNCWatchDog(1000, networkmanager, Motor1, Motor2)
{};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    
    //any other setup goes here
    
    Buck.setup();
    Motor1.setup();
    Motor2.setup();

    canbus.setup();
    
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    // Defining these so the methods following are less ugly
    uint8_t motorservice1 = (uint8_t) Services::ID::Motor1;
    uint8_t motorservice2 = (uint8_t) Services::ID::Motor2;

    networkmanager.addInterface(&canbus);

    networkmanager.registerService(motorservice1,Motor1.getThisNetworkCallback());
    networkmanager.registerService(motorservice2,Motor2.getThisNetworkCallback());
    
};

void System::systemUpdate(){
    Buck.update();
    // WatchDog.update();
};
