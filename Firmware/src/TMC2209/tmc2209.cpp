#include "tmc2209.h"
#include "tmcTelemPacket.h"

#include "Config/services_config.h"

#include <Arduino.h>

void TMC2209::setup() {
    // pinMode(PinMap::StepperDiag, OUTPUT);
    // pinMode(PinMap::StepperIndex, OUTPUT);

    pinMode(m_stepperStepPin, OUTPUT);
    pinMode(m_stepperDirPin, OUTPUT);

    // pinMode(PinMap::StepperRx, INPUT);
    // pinMode(PinMap::StepperTx, OUTPUT);

    m_tmcTelem.header.type = static_cast<uint8_t>(78);
	m_tmcTelem.header.source = m_networkmanager.getAddress();
	m_tmcTelem.header.source_service = static_cast<uint8_t>(Services::ID::TMC);
}

void TMC2209::update() {
    m_log.armed = m_armed;
    m_log.state = static_cast<uint32_t>(m_stepperCommand);

    if (m_stepperCommand == StepperCommandPayload::HOLD) {
        return;
    }

    uint64_t currentTimeUs = micros();

    if (m_stepperCommand != StepperCommandPayload::HOLD && (currentTimeUs - m_lastStepTimeUs > HalfStepDelayUs)) {
        m_lastStepTimeUs = currentTimeUs;

        digitalWrite(m_stepperStepPin, m_currentStepState);
        m_currentStepState = !m_currentStepState;
    }
}

void TMC2209::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr) {
    if (static_cast<uint32_t>(commandID) == 8) {
        SimpleCommandPacket telemRequest(*packetptr);

        m_tmcTelem.header.destination = telemRequest.header.source;
        m_tmcTelem.header.destination_service = telemRequest.header.source_service;

        m_tmcTelem.armed = m_log.armed;
        m_tmcTelem.status = m_log.state;
        m_tmcTelem.system_time = millis();

        m_networkmanager.sendPacket(m_tmcTelem);
        return;
    }

    switch (commandID) {
    case NRCPacket::NRC_COMMAND_ID::EXECUTE: {
        if (!m_armed) {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Service not armed, cannot execute");
            execute_impl(std::move(packetptr));
        }
        break;
    }
    case NRCPacket::NRC_COMMAND_ID::ARM: {
        m_armed = true;
        break;
    }
    case NRCPacket::NRC_COMMAND_ID::DISARM: {
        m_armed = false;
        break;
    }
    default:
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Invalid command id");
        break;
    }
}

void TMC2209::execute_base(int32_t arg) {
    switch (arg) {
    case static_cast<int32_t>(StepperCommandPayload::HOLD): {
        m_stepperCommand = StepperCommandPayload::HOLD;
        break;
    }
    case static_cast<int32_t>(StepperCommandPayload::EXTEND): {
        m_stepperCommand = StepperCommandPayload::EXTEND;
        setExtend();
        break;
    }
    case static_cast<int32_t>(StepperCommandPayload::RETRACT): {
        m_stepperCommand = StepperCommandPayload::RETRACT;
        setRetract();
        break;
    }
    default: {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Invalid execute arg for TMC");
        break;
    }
    }
}

void TMC2209::setExtend() {
    digitalWrite(m_stepperDirPin, ExtendDirection);
}

const TMC2209::Log& TMC2209::getLog() {
    return m_log;
}

void TMC2209::setRetract() {
    digitalWrite(m_stepperDirPin, RetractDirection);
}
