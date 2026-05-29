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
Buck(systemstatus,PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 1500, 470),
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
m_servo0_pwm(PinMap::ServoPWM0),
m_servo1_pwm(PinMap::ServoPWM1),
m_servo0(m_servo0_pwm, networkmanager, "Srvo0", 0,0,100,1100,1940,0,100), //bottom servo
m_servo1(m_servo1_pwm, networkmanager, "Srvo1", 0,0,100,1100,1940,0,100) //top servo

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

    m_servo0.setup(false);
    m_servo1.setup(false);
    canbus.setup(); 

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    // Defining these so the methods following are less ugly
    uint8_t servo0service = static_cast<uint8_t>(Services::ID::Servo0);
    uint8_t servo1service = static_cast<uint8_t>(Services::ID::Servo1);

    networkmanager.addInterface(&canbus);

    networkmanager.registerService(servo0service,m_servo0.getThisNetworkCallback());
    networkmanager.registerService(servo1service,m_servo1.getThisNetworkCallback());
    
};

void System::systemUpdate(){
    Buck.update();
    // clifford.update();
}
