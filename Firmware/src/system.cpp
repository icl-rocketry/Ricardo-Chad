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
#include "Commands/packets/ChadPotsPacket.h"

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
pot2("Potentiometer2", PinMap::Pot2Control, 0, 1),
pot3("Potentiometer3", PinMap::Pot3Control, 0, 1)
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

    //Setup for Potentiometers (max, low, min) in mV
    pot0.setup(3300, 0, 0);
    pot0.setSampleRate(PotSampleRate);
    pot1.setup(3300, 0, 0);
    pot1.setSampleRate(PotSampleRate);
    pot2.setup(3300, 0, 0);
    pot2.setSampleRate(PotSampleRate);
    pot3.setup(3300, 0, 0);
    pot3.setSampleRate(PotSampleRate);
    //RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Samplerate:" + std::to_string(PotSampleRate));


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
    sendInfoPacket = false;
    
    // Get potentiometer readings

    pot0.update(Pot0OutputV);
    Pot0OutputV = static_cast<float> (alpha*Pot0OutputV + (1-alpha)*Pot0OutputVOld); // Simple low pass filter to smooth out voltage readings
    Pot0OutputVOld = Pot0OutputV;

    pot1.update(Pot1OutputV);
    Pot1OutputV = static_cast<float> (alpha*Pot1OutputV + (1-alpha)*Pot1OutputVOld); // Simple low pass filter to smooth out voltage readings
    Pot1OutputVOld = Pot1OutputV;

    pot2.update(Pot2OutputV);
    Pot2OutputV = static_cast<float> (alpha*Pot2OutputV + (1-alpha)*Pot2OutputVOld); // Simple low pass filter to smooth out voltage readings
    Pot2OutputVOld = Pot2OutputV;

    pot3.update(Pot3OutputV);
    Pot3OutputV = static_cast<float> (alpha*Pot3OutputV + (1-alpha)*Pot3OutputVOld); // Simple low pass filter to smooth out voltage readings
    Pot3OutputVOld = Pot3OutputV;

    // Calculate potentiometer percentage based on voltage readings and thresholds

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
        Pot0PercentageOld = Pot0Percentage;
        sendInfoPacket = true;

        // Send command packet to self for servo actuation
        SimpleCommandPacket actuate_servo(2, Pot0Percentage);
	    actuate_servo.header.source = networkmanager.getAddress();
	    actuate_servo.header.source_service = static_cast<uint8_t>(Services::ID::Potentiometers);
	    actuate_servo.header.destination = networkmanager.getAddress();
	    actuate_servo.header.destination_service = static_cast<uint8_t>(Services::ID::Servo0);
	    actuate_servo.header.uid = static_cast<uint8_t>(68);
        networkmanager.sendPacket(actuate_servo);
    }
    
    if (Pot1OutputV <= PotLowerVThreshhold) {
        Pot1Percentage = 0;
    }
    else if (Pot1OutputV >= PotUpperVThreshhold) {
        Pot1Percentage = 100;
    }
    else {
        Pot1PercentageRaw = ((Pot1OutputV-PotLowerVThreshhold)/(PotUpperVThreshhold-PotLowerVThreshhold))*100;
        Pot1Percentage = std::min(std::max(static_cast<int>(Pot1PercentageRaw), 0), 100);
    }    
    if (Pot1Percentage != Pot1PercentageOld) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pot1 Voltage: " + std::to_string(Pot1OutputV) + "mV, " + std::to_string(Pot1Percentage) + "%\n");
        Pot1PercentageOld = Pot1Percentage;
        sendInfoPacket = true;
        
        // Send command packet to self for servo actuation
        SimpleCommandPacket actuate_servo(2, Pot0Percentage);
	    actuate_servo.header.source = networkmanager.getAddress();
	    actuate_servo.header.source_service = static_cast<uint8_t>(Services::ID::Potentiometers);
	    actuate_servo.header.destination = networkmanager.getAddress();
	    actuate_servo.header.destination_service = static_cast<uint8_t>(Services::ID::Servo1);
	    actuate_servo.header.uid = static_cast<uint8_t>(69);
        networkmanager.sendPacket(actuate_servo);
    }

    if (Pot2OutputV <= PotLowerVThreshhold) {
        Pot2Percentage = 0;
    }
    else if (Pot2OutputV >= PotUpperVThreshhold) {
        Pot2Percentage = 100;
    }
    else {
        Pot2PercentageRaw = ((Pot2OutputV-PotLowerVThreshhold)/(PotUpperVThreshhold-PotLowerVThreshhold))*100;
        Pot2Percentage = std::min(std::max(static_cast<int>(Pot2PercentageRaw), 0), 100);
    }    
    if (Pot2Percentage != Pot2PercentageOld) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pot2 Voltage: " + std::to_string(Pot2OutputV) + "mV, " + std::to_string(Pot2Percentage) + "%\n");
        Pot2PercentageOld = Pot2Percentage;
        sendInfoPacket = true;

        // Send command packet to Masters Servo for actuation
        SimpleCommandPacket actuate_servo(2, Pot2Percentage);
        actuate_servo.header.source = networkmanager.getAddress();
        actuate_servo.header.source_service = static_cast<uint8_t>(Services::ID::Potentiometers);
        actuate_servo.header.destination = static_cast<uint8_t>(101);
        actuate_servo.header.destination_service = static_cast<uint8_t>(10);
        actuate_servo.header.uid = static_cast<uint8_t>(70);
        networkmanager.sendPacket(actuate_servo);
    }

    if (Pot3OutputV <= PotLowerVThreshhold) {
        Pot3Percentage = 0;
    }
    else if (Pot3OutputV >= PotUpperVThreshhold) {
        Pot3Percentage = 100;
    }
    else {
        Pot3PercentageRaw = ((Pot3OutputV-PotLowerVThreshhold)/(PotUpperVThreshhold-PotLowerVThreshhold))*100;
        Pot3Percentage = std::min(std::max(static_cast<int>(Pot3PercentageRaw), 0), 100);
    }    
    if (Pot3Percentage != Pot3PercentageOld) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pot3 Voltage: " + std::to_string(Pot3OutputV) + "mV, " + std::to_string(Pot3Percentage) + "%\n");
        Pot3PercentageOld = Pot3Percentage;
        sendInfoPacket = true;

        // Send command packet to Masters Servo for actuation
        SimpleCommandPacket actuate_servo(2, Pot3Percentage);
        actuate_servo.header.source = networkmanager.getAddress();
        actuate_servo.header.source_service = static_cast<uint8_t>(Services::ID::Potentiometers);
        actuate_servo.header.destination = static_cast<uint8_t>(101);
        actuate_servo.header.destination_service = static_cast<uint8_t>(11);
        actuate_servo.header.uid = static_cast<uint8_t>(71);
        networkmanager.sendPacket(actuate_servo);
    }

    // Send potentiometer packet if any of the percentages have changed
    if (sendInfoPacket) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Sending Potentiometer Packet\n");
        ChadPotsPacket chadpots;
        
        chadpots.header.type = static_cast<uint8_t>(103);
	    chadpots.header.source = networkmanager.getAddress();
	    chadpots.header.source_service = static_cast<uint8_t>(Services::ID::Potentiometers);
	    chadpots.header.destination = static_cast<uint8_t>(101);
	    chadpots.header.destination_service = static_cast<uint8_t>(2);
	    chadpots.header.uid = static_cast<uint8_t>(67);
        chadpots.Pot0Percentage = Pot0Percentage;
        chadpots.Pot1Percentage = Pot1Percentage;
        chadpots.Pot2Percentage = Pot2Percentage;
        chadpots.Pot3Percentage = Pot3Percentage;

        networkmanager.sendPacket(chadpots);
    }

}