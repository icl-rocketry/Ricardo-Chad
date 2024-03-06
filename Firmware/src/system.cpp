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


System::System():
RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
Buck(PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 1500, 470),
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
Servo1(PinMap::ServoPWM1, 0, networkmanager,90,0,180,0,180,1000,2000),
Servo2(PinMap::ServoPWM2, 1, networkmanager,90,0,180,0,180,1000,2000)
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
    Servo1.setup();
    Servo2.setup();
    //override any callibration that has been preivously set and stored
    Servo1.setAngleLims(0,180);
    Servo2.setAngleLims(0,180);
    Servo1.setHome(90);
    Servo2.setHome(90);
    Servo1.reset();
    Servo2.reset();


    canbus.setup();
    
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    //Defining these so the methods following are less ugly
    uint8_t servoservice1 = (uint8_t) Services::ID::Servo1;
    uint8_t servoservice2 = (uint8_t) Services::ID::Servo2;

    networkmanager.addInterface(&canbus);

    networkmanager.registerService(servoservice1,Servo1.getThisNetworkCallback());
    networkmanager.registerService(servoservice2,Servo2.getThisNetworkCallback());
    
};

long prevTime = 0;
bool update = false;

void System::systemUpdate(){
    Buck.update();
};