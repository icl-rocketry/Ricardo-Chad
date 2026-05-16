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
m_servo0(m_servo0_pwm, networkmanager, "Srvo0"),
m_servo1(m_servo1_pwm, networkmanager, "Srvo1"),
pot0("Potentiometer0", PinMap::Pot0Control, 0, 1),
pot1("Potentiometer1", PinMap::Pot1Control, 0, 1),
i2cBus(1),
display(U8G2_R0, PinMap::sclPin, PinMap::sdaPin, U8X8_PIN_NONE, PinMap::dcPin, PinMap::resetPin)
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

    m_servo0.setup();
    m_servo1.setup();
    canbus.setup(); 

    //Setup for Potentiometers and Switches
    pot0.setup(3300, 0, 0);
    pot0.setSampleRate(PotSampleRate);
    pot1.setup(3300, 0, 0);
    pot1.setSampleRate(PotSampleRate);
    //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Samplerate:" + std::to_string(PotSampleRate));

    //Setup for display
    display.begin();
    display.clearBuffer();
  
    // Set text properties
    display.setFont(u8g2_font_10x20_tf);
  
    // Print Base display
    display.setDrawColor(10);
    display.drawLine(0,32,255,32);
    display.drawLine(64,0,64,63);
    display.drawLine(128,0,128,63);
    display.drawLine(192,0,192,63);
    display.setDrawColor(15);

    display.drawStr(10, 20, "POT1");
    display.drawStr(74, 20, "POT2");
    display.drawStr(138, 20, "POT3");
    display.drawStr(202, 20, "POT4");

  
    // IMPORTANT: You must call .sendBeffer() to actually show the buffer on the screen
    display.sendBuffer();

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    //Defining these so the methods following are less ugly
    uint8_t servoservice0 = static_cast<uint8_t>(Services::ID::Servo0);
    uint8_t servoservice1 = static_cast<uint8_t>(Services::ID::Servo1);

    networkmanager.addInterface(&canbus);

    networkmanager.registerService(servoservice0,m_servo0.getThisNetworkCallback());
    networkmanager.registerService(servoservice1,m_servo1.getThisNetworkCallback());
    
};

void System::systemUpdate(){
    Buck.update();
    
    pot0.update(Pot0OutputV);
    Pot0OutputV = static_cast<float> (alpha*Pot0OutputV + (1-alpha)*Pot0OutputVOld); // Simple low pass filter to smooth out voltage readings
    Pot0OutputVOld = Pot0OutputV;

    if (Pot0OutputV <= PotLowerVThreshhold) {
        Pot0Percentage = 0;
    }
    else if (Pot0OutputV >= PotUpperVThreshhold) {
        Pot0Percentage = 100;
    }
    else {
        Pot0PercentageRaw = ((Pot0OutputV-PotLowerVThreshhold)/(PotUpperVThreshhold-PotLowerVThreshhold))*100;
        Pot0Percentage = std::min(std::max(static_cast<int>(Pot0PercentageRaw), 0), 100);
    }    
   
    if (Pot0Percentage != Pot0PercentageOld) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pot0 Voltage: " + std::to_string(Pot0OutputV) + "mV, " + std::to_string(Pot0Percentage) + "%\n");
        
        // display.setDrawColor(0);
        // display.drawBox(0,33, 64, 30);
        // display.setDrawColor(15);

        // display.setCursor(10,52);
        // display.print(Pot0Percentage);
        // display.print("%");
        // display.sendBuffer();
        Pot0PercentageOld = Pot0Percentage;
    }
    
}