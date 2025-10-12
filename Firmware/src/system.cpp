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

    // The FTS signal lines are pulled up externally so these should also be pullups
    pinMode(PinMap::FTSSignal0, INPUT_PULLUP);
    pinMode(PinMap::FTSSignal1, INPUT_PULLUP);

    // Delay to allow inputs to normalize
    delay(1000);

    RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("\n\n----- FTS Active -----\n\n");
};

void System::systemUpdate(){
    // Read the cable into pinSignal.
    int ftsSignal0 = digitalRead(PinMap::FTSSignal0);
    int ftsSignal1 = digitalRead(PinMap::FTSSignal1);

    // FTS Active
    // Ensure CAN is not spammed
    static int lastTime = 0;
    static const int commandTimeoutMs = 50;

    if (ftsSignal0 == LOW && ftsSignal1 == LOW && millis() - lastTime > commandTimeoutMs) {
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