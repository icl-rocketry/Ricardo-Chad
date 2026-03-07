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
//lcd(0x27,16,2)
display(128, 32, &Wire, -1)
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

    //Setup for display
    //i2cBus.begin(PinMap::sdaPin, PinMap::sclPin, uint32_t(100000)); // Default I2C frequency is 100kHz
    Wire.begin(PinMap::sdaPin, PinMap::sclPin, 100000);

    // Initialize with the I2C addr 0x3C
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    }

    display.clearDisplay();
  
    // Set text properties
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
  
    // Print "Hello, ESP32!"
    display.setCursor(0, 0);
    display.println("Potentiometer:");
  
    // IMPORTANT: You must call display() to actually show the buffer on the screen
    display.display();

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

    if (Pot0OutputV < PotLowerVThreshhold) {
        Pot0Percentage = 0;
    }
    else if (Pot0OutputV > PotUpperVThreshhold) {
        Pot0Percentage = 100;
    }
    else {
        Pot0Percentage = static_cast<int>(((Pot0OutputV-PotLowerVThreshhold)/(PotUpperVThreshhold-PotLowerVThreshhold))*100);
    }    
    
    if (Pot0Percentage != Pot0PercentageOld) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pot0 Voltage: " + std::to_string(Pot0OutputV) + "mV, " + std::to_string(Pot0Percentage) + "%\n");
        //lcd.setCursor(0,0); lcd.print("P0: "); lcd.print(Pot0Percentage); lcd.print("%   ");
        display.setCursor(0,15);
        display.fillRect(0,10,64,15, SSD1306_BLACK);
        display.print(Pot0Percentage);
        display.display();
        Pot0PercentageOld = Pot0Percentage;
    }
    
}