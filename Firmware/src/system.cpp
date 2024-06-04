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
Motor1(PinMap::ServoPWM1, 0, networkmanager,0,0,100,0,100,NRCRemoteServo::counts(1130),NRCRemoteServo::counts(2000)),
Motor2(PinMap::ServoPWM2, 1, networkmanager,0,0,100,0,100,NRCRemoteServo::counts(1130),NRCRemoteServo::counts(2000))
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

    //Defining these so the methods following are less ugly
    uint8_t motorservice1 = (uint8_t) Services::ID::Motor1;
    uint8_t motorservice2 = (uint8_t) Services::ID::Motor2;

    networkmanager.addInterface(&canbus);

    networkmanager.registerService(motorservice1,Mo1or2.getThisNetworkCallback());
    networkmanager.registerService(motorservice2,Motor2.getThisNetworkCallback());
    
};

long prevTime = 0;
bool update = false;

void System::systemUpdate(){
    //everything under here is done by arlo, idk if it works!!!
     if (this -> _state.flagSet(COMPONENT_STATUS_FLAGS::DISARMED))
    {
        current = GNCState::Default;
    }

    switch (currentGNCState)
    {

        case GNCState::Default:
        {
            //make code so nothing runs ie disarmed 
            //set Servo1 power = 0 
            //set Servo2 power = 0
            //set motor power = 0 
        }
        case GNCState::Arm:
        {
            //set it so TVC moves to pointing down but the motor cant move 
        }
        case GNCState::Hover:
        {
            //case for hovering (TVC and motor on)
            //add exit conditions so if it goes over a specific boundary go back to abort state ie kill
            
        }
        case GNCState::Abort: 
        {
            //cut power 
        }
       
    }


    Buck.update();
};