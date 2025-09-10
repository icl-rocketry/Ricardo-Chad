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
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3)
{};

void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    
    canbus.setup(); 

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    networkmanager.addInterface(&canbus);

    // FTSSignal pin is default low, so need a pulldown
    pinMode(PinMap::FTSSignal, INPUT_PULLDOWN);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("FTS Active");
};

void System::systemUpdate(){
    // Do some basic debouncing here
    static int ftsSignal = LOW;
    static int pinSignal = LOW;
    static const int debounceMs = 20;
    static int debounceStart = 0;

    // Read the cable into pinSignal.
    pinSignal = digitalRead(PinMap::FTSSignal);

    // If the read signal is different than the saved and it hasn't already
    // been detected then start a timer.
    if (pinSignal != ftsSignal && debounceStart == 0) {
        debounceStart = millis();

    // Else if the read signal is back to the same then clear the timeout.
    } else if (pinSignal == ftsSignal) {
        debounceStart = 0;
    }

    // If the timer is set and it has run out then swap saved pin signal.
    if (debounceStart != 0 && millis() - debounceStart > debounceMs) {
        ftsSignal = pinSignal;
        debounceStart = 0;
    }

    // FTS Active
    // Ensure CAN is not spammed
    static int lastTime = std::numeric_limits<int>::min();
    static const int commandTimeoutMs = 100;

    if (ftsSignal == HIGH && millis() - lastTime > commandTimeoutMs) {
        ftsDeployed = true;

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("FTS Deployed");

        SimpleCommandPacket ftsCommand(static_cast<uint8_t>(Commands::ID::FTSActive), 0);
        ftsCommand.header.type = static_cast<uint8_t>(NRCPacket::TYPES::NRC_COMMAND);
        ftsCommand.header.source = networkmanager.getAddress();
        ftsCommand.header.source_service = 2;       // Command Service
        ftsCommand.header.destination = 2;          // Pickle Address
        ftsCommand.header.destination_service = 2;  // Command Service

        networkmanager.sendPacket(ftsCommand);

        lastTime = millis();
    }
}