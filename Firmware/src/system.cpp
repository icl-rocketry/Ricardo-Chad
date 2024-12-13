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

#include "TVC/TVCController.h"


const int TX_uart = 10;
const int RX_uart = 9;

std::unique_ptr<TVCController> controller;
// Odrive UART RX = 1, TX = 2

System::System():
RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
Buck(systemstatus,PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 1500, 470),
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
m_servo0_pwm(PinMap::ServoPWM0, 0),
m_servo1_pwm(PinMap::ServoPWM1, 1),
m_servo0(m_servo0_pwm, networkmanager, "Srvo0"),
m_servo1(m_servo1_pwm, networkmanager, "Srvo1")
{};

#define log(x) RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(x)

void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    
    //any other setup goes here
    
    Buck.setup();

    m_servo0.setup();
    m_servo1.setup();
    canbus.setup();
    
    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    //Defining these so the methods following are less ugly
    uint8_t servoservice0 = static_cast<uint8_t>(Services::ID::Servo0);
    uint8_t servoservice1 = static_cast<uint8_t>(Services::ID::Servo1);

    networkmanager.addInterface(&canbus);

    networkmanager.registerService(servoservice0,m_servo0.getThisNetworkCallback());
    networkmanager.registerService(servoservice1,m_servo1.getThisNetworkCallback());

    delay(1000);

    pinMode(7, INPUT_PULLDOWN);
    log("Initialising!");

    Serial1.begin(115200, SERIAL_8N1, RX_uart, TX_uart);

    while (!Serial1) { delay(10); }


    controller = std::make_unique<TVCController>(networkmanager, Serial1);
    controller->arm_base(0);
    controller->execute_base(0x1);
    log("Executing!");
}

void System::systemUpdate(){
    // Buck.update();
    controller->update();
}