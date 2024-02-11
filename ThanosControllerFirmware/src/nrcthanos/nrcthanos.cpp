#include "nrcthanos.h"
#include <math.h>
#include <Arduino.h>

#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/services_config.h"

void NRCThanos::setup()
{
    fuelServo.setup();
    oxServo.setup();

    fuelServo.setAngleLims(0, 175);
    oxServo.setAngleLims(0, 160);

    m_oxThrottleRange = 160 - oxServoPreAngle;
    m_fuelThrottleRange = 175 - fuelServoPreAngle;

    pinMode(_overrideGPIO, INPUT_PULLUP);
}

void NRCThanos::update()
{
    // Close valves if component disarmed
    if (this->_state.flagSet(COMPONENT_STATUS_FLAGS::DISARMED))
    {
        currentEngineState = EngineState::Default;
        // _Buck.restart(5); // abuse restart command to prevent servos from getting too hot when in disarmed state
    }

    // Close valves if abort is used
    if (digitalRead(_overrideGPIO) == 1)
    {
        currentEngineState = EngineState::ShutDown;
    }

    // Close valves after a flat 14 seconds
    if ((millis() - ignitionTime > m_cutoffTime) && _ignitionCalls > 0)
    {
        currentEngineState = EngineState::ShutDown;
        _ignitionCalls = 0;
    }

    switch (currentEngineState)
    {

    case EngineState::Default:
    {
        fuelServo.goto_Angle(0);
        oxServo.goto_Angle(0);
        _polling = false;
        break;
    }

    case EngineState::Ignition:

    { // ignition sequence
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Ignition state");
        if (timeFrameCheck(pyroFires, fuelValvePreposition))
        {
            firePyro(fuelValvePreposition - pyroFires);
        }

        else if (timeFrameCheck(fuelValvePreposition, oxValvePreposition))
        {
            fuelServo.goto_Angle(fuelServoPreAngle);
        }

        else if (timeFrameCheck(oxValvePreposition, endOfIgnitionSeq))
        {
            oxServo.goto_Angle(oxServoPreAngle);
        }

        else if (timeFrameCheck(endOfIgnitionSeq))
        {
            currentEngineState = EngineState::NominalT;
            m_nominalEntry = millis();
            m_firstNominal = true;
            resetVars();
        }

        break;
    }

    case EngineState::NominalT:
    {
        fuelServo.goto_Angle(150);
        oxServo.goto_Angle(150);
    }

    case EngineState::ShutDown:
    {
        fuelServo.goto_Angle(0);
        oxServo.goto_Angle(0);
        _polling = false;

        break;
    }

    default:
    {
        break;
    }
    }
}

bool NRCThanos::nominalEngineOp()
{
    if (_chamberP > 5 || _chamberP < 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void NRCThanos::updateChamberP(float chamberP)
{
    lastTimeChamberPUpdate = millis();
    _chamberP = chamberP;
}

void NRCThanos::updateThrust(float thrust)
{
    lastTimeThrustUpdate = millis();
    _thrust = abs(thrust);
}

void NRCThanos::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        currentEngineState = EngineState::Ignition;
        ignitionTime = millis();
        _ignitionCalls = 0;
        resetVars();
        _polling = true;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Ignition");
        break;
    }
    case 2:
    {
        currentEngineState = EngineState::ShutDown;
        _polling = false;
        _ignitionCalls = 0;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        _polling = false;
        currentEngineState = EngineState::Debug;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    }
}

void NRCThanos::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr)
{
    SimpleCommandPacket command_packet(*packetptr);
    switch (static_cast<uint8_t>(commandID))
    {
    case 6:
    {
        if (currentEngineState == EngineState::Debug)
        {
            fuelServo.goto_Angle(command_packet.arg);
            break;
        }
        else
        {
            break;
        }
    }
    case 7:
    {
        if (currentEngineState == EngineState::Debug)
        {
            oxServo.goto_Angle(command_packet.arg);
        }
        else
        {
            break;
        }
    }
    default:
    {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
        break;
    }
    }
}

bool NRCThanos::timeFrameCheck(int64_t start_time, int64_t end_time)
{
    if (millis() - ignitionTime > start_time && end_time == -1)
    {
        return true;
    }

    else if (millis() - ignitionTime > start_time && millis() - ignitionTime < end_time)
    {
        return true;
    }

    else
    {
        return false;
    }
}

void NRCThanos::gotoWithSpeed(NRCRemoteServo &Servo, uint16_t demandAngle, float speed, float &prevAngle, float &currAngle, uint32_t &prevUpdateT)
{
    if (millis() - prevUpdateT < 10)
    {
        return;
    }

    if (prevUpdateT == 0)
    {
        prevUpdateT = millis();
        return;
    }

    float timeSinceLast = (float)(millis() - prevUpdateT) / 1000.0; // in seconds;

    if ((demandAngle - prevAngle) > 0)
    {
        currAngle = prevAngle + (timeSinceLast * speed);
    }
    else if ((demandAngle - prevAngle) < 0)
    {
        currAngle = prevAngle - (timeSinceLast * speed);
    }
    else
    {
        currAngle = currAngle;
    }

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(currAngle));
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(_thrust));
    // Servo.goto_Angle(static_cast<uint16_t>(currAngle));
    Servo.goto_AngleHighRes(currAngle);
    prevAngle = currAngle;

    prevUpdateT = millis();
}

void NRCThanos::gotoThrust(float target, float closespeed, float openspeed)
{
    // if ((target * 1.02 > _thrust || target * 0.98 < _thrust) && !m_thrustreached)
    // {
    //     m_timeThrustreached = millis();
    //     m_thrustreached = true;
    // }

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(target));
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(_thrust));

    if (target * 1.02 < _thrust)
    {
        gotoWithSpeed(oxServo, 70, closespeed, m_oxServoPrevAngle, m_oxServoCurrAngle, m_oxServoPrevUpdate);
    }
    else if (_thrust < target * 0.98)
    {
        gotoWithSpeed(oxServo, 180, openspeed, m_oxServoPrevAngle, m_oxServoCurrAngle, m_oxServoPrevUpdate);
    }
    else
    {
        oxServo.goto_Angle(m_oxServoCurrAngle);
    }

    m_oxPercent = (float)(m_oxServoCurrAngle - oxServoPreAngle) / (float)(m_oxThrottleRange);
    m_fuelPercent = m_oxPercent + m_fuelExtra;
    float fuelAngle = (float)(m_fuelPercent * m_fuelThrottleRange) + fuelServoPreAngle;

    if (fuelAngle < fuelServoPreAngle)
    {
        fuelServo.goto_AngleHighRes(fuelServoPreAngle);
    }
    else
    {
        fuelServo.goto_AngleHighRes(fuelAngle);
    }
}

void NRCThanos::firePyro(uint32_t duration)
{
    if (millis() - _prevFiring > _ignitionCommandSendDelta)
    {
        if (_ignitionCalls < _ignitionCommandMaxCalls)
        {
            SimpleCommandPacket ignition_command(2, duration);
            ignition_command.header.source_service = static_cast<uint8_t>(Services::ID::Thanos);
            ignition_command.header.destination_service = m_ingitionService;
            ignition_command.header.source = _address;
            ignition_command.header.destination = m_ignitionNode;
            ignition_command.header.uid = 0;
            _networkmanager.sendPacket(ignition_command);
            _prevFiring = millis();
            _ignitionCalls++;
        }
    }
}

bool NRCThanos::pValUpdated()
{
    if ((millis() - lastTimeChamberPUpdate) > pressureUpdateTimeLim || (millis() - lastTimeThrustUpdate) > pressureUpdateTimeLim)
    {
        return false;
    }
    else
    {
        return true;
    }
}