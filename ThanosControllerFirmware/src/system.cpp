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

#include <librrc/rocketcomponent.h>

System::System():
RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
Buck(PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 1500, 470),
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
chamberPTap(1, GeneralConfig::Kermitaddr, static_cast<uint8_t>(Services::ID::chamberPTap), static_cast<uint8_t>(Services::ID::chamberPTap), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
thrustGauge(2, GeneralConfig::Kermitaddr, static_cast<uint8_t>(Services::ID::thrustGauge), static_cast<uint8_t>(Services::ID::thrustGauge), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
chamberPTapPoller(50, &chamberPTap),
thrustGaugePoller(20, &thrustGauge),
Thanos(networkmanager,PinMap::ServoPWM1,0,PinMap::ServoPWM2,1,PinMap::EngineOverride,PinMap::TVCPIN0,PinMap::TVCPIN1,PinMap::TVCPIN2,networkmanager.getAddress(),Buck)
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
    chamberPTapPoller.setup();
    thrustGaugePoller.setup();
    Thanos.setup();
    canbus.setup();
    networkmanager.addInterface(&canbus);

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    //Defining these so the methods following are less ugly
    uint8_t thanosservice = static_cast<uint8_t>(Services::ID::Thanos);
    uint8_t chamberPTapservice = static_cast<uint8_t>(Services::ID::chamberPTap);
    uint8_t thrustGaugeservice = static_cast<uint8_t>(Services::ID::thrustGauge);

    networkmanager.registerService(thanosservice,Thanos.getThisNetworkCallback());
    networkmanager.registerService(chamberPTapservice,[this](packetptr_t packetptr){chamberPTap.networkCallback(std::move(packetptr));});
    networkmanager.registerService(thrustGaugeservice,[this](packetptr_t packetptr){thrustGauge.networkCallback(std::move(packetptr));});
};

void System::systemUpdate(){
    Buck.update();

    if(Thanos.getPollingStatus()){  
        chamberPTapPoller.update();
        thrustGaugePoller.update();
    }
    
    if(chamberPTapPoller.newdata)
    {
        Thanos.updateChamberP(chamberPTapPoller.getVal());
    }

    if(thrustGaugePoller.newdata)
    {
        Thanos.updateThrust(thrustGaugePoller.getVal());
    }

    Thanos.update();
};