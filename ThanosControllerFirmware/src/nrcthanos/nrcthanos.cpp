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
    pinMode(_tvcpin0, OUTPUT);
    pinMode(_tvcpin1, OUTPUT);
    pinMode(_tvcpin2, OUTPUT);
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

    // // Close valves after a flat 14 seconds
    if ((millis() - ignitionTime > m_cutoffTime) && _ignitionCalls > 0)
    {
        currentEngineState = EngineState::ShutDown;
        _ignitionCalls = 0;
    }

    // Close ox fill 10.5s after ignition
    if ((millis() - ignitionTime > m_oxFillCloseTime) && _ignitionCalls > 0)
    {
        closeOxFill();
    }

    thanosStateMachine();
    odriveStateMachine();
}

void NRCThanos::thanosStateMachine()
{
    switch (currentEngineState)
    {

    case EngineState::Default:
    {
        fuelServo.goto_Angle(0);
        oxServo.goto_Angle(0);
        if (this->_state.flagSet(COMPONENT_STATUS_FLAGS::NOMINAL) && m_calibrationDone)
        {
            // currOdriveState = oDriveState::Idle; // allow pwm stuff to be reset
            currOdriveState = oDriveState::Armed;
            // _Buck.restart(5); // abuse restart command to prevent servos from getting too hot when in disarmed state
        }
        else
        {
            currOdriveState = oDriveState::Idle;
        }

        _polling = false;
        break;
    }

    case EngineState::Ignition:

    { // ignition sequence
        // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Ignition state");
        if (timeFrameCheck(pyroFires, fuelValveNominal))
        {
            firePyro(fuelValveNominal - pyroFires);
        }
        else if (timeFrameCheck(fuelValveNominal, oxValveNominal))
        {
            openOxFill();
            //fuelServo.goto_Angle(fuelNominalAngle);
            gotoWithSpeed(fuelServo, fuelNominalAngle, m_firstNominalSpeed, m_fuelServoPrevAngle, m_fuelServoCurrAngle, m_fuelServoPrevUpdate);
            m_nominalEntry = millis();
        }
        else if (timeFrameCheck(oxValveNominal, -1))
        {
            //oxServo.goto_Angle(oxNominalAngle);
            gotoWithSpeed(oxServo, oxNominalAngle, m_firstNominalSpeed, m_oxServoPrevAngle, m_oxServoCurrAngle, m_oxServoPrevUpdate);
            currentEngineState = EngineState::NominalT;
            //currOdriveState = oDriveState::Armed;     // already arms when we call arm_impl
            m_firstNominal = true;
            resetVars();
        }

        break;
    }

    case EngineState::NominalT:
    {
        gotoChamberP(m_targetChamberP);   // throttling based on chamber pressure value
        // gotoFuelP(m_targetFuelP);

        if (millis() - m_nominalEntry > m_startTVCCircle && !m_tvcEntered)   // delay before transitioning to TVC sequence
        {
            currOdriveState = oDriveState::HotfireProfile;
            m_tvcEntry = millis();
            m_firstNominal = 0;
            m_tvcEntered = true;
        }

        // if (millis() - m_tvcEntry > m_tvctime)  // tvc sequence timeout
        // {
        //     currOdriveState = oDriveState::Armed;  
        // }

        break;
    }

    case EngineState::Calibration:
    {
        currOdriveState = oDriveState::Calibration;
        if (millis() - m_calibrationStart > m_calibrationTime)
        {
            currentEngineState = EngineState::Default;
            m_calibrationDone = 1;
        }

        break;
    }

    // case EngineState::TVCCircle:
    // {
    //     // fuelServo.goto_Angle(fuelNominalAngle);
    //     // oxServo.goto_Angle(oxNominalAngle);
    //     gotoChamberP(m_targetChamberP, m_servoFast, m_servoFast);   // throttling based on chamber pressure value
    //     currOdriveState = oDriveState::HotfireProfile;

    //     if (millis() - m_tvcEntry > m_tvctime)  // tvc sequence timeout
    //     {
    //         //currentEngineState = EngineState::NominalT;
    //         currOdriveState = oDriveState::Armed;  
    //     }
    //     break;
    // }

    case EngineState::ShutDown:
    {
        fuelServo.goto_Angle(0);
        oxServo.goto_Angle(0);
        closeOxFill();
        //currOdriveState = oDriveState::Armed;      // lock in neutral position
        currOdriveState = oDriveState::Idle; 
        _polling = false;
        break;
    }

    default:
    {
        break;
    }
    }
}

void NRCThanos::odriveStateMachine(){
    switch(currOdriveState){
        case oDriveState::Idle:{
            motorsOff();
            break;
        }
        case oDriveState::Armed:{
            motorsArmed();
            break;
        }
        case oDriveState::LockedCurrent:{
            motorsLocked();
            break;
        }
        case oDriveState::HotfireProfile:{
            motorsCircle();
            break;
        }
        case oDriveState::Calibration:{
            motorsCalibrate();
            break;
        }
        case oDriveState::Debug:{
            motorsDebug();
            break;
        }
        case oDriveState::pwmEnable:{
            pwmEnable();
            break;
        }
        // default:
        // {
        //     motorsOff();
        //     break;
        // }
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

void NRCThanos::updateFuelP(float fuelP)
{
    lastTimeFuelPUpdate = millis();
    _fuelP = fuelP;

}

void NRCThanos::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1:
    {
        if (currentEngineState != EngineState::Default || !m_calibrationDone) 
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
    case 4:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        _polling = false;
        currentEngineState = EngineState::Calibration;
        m_calibrationStart = millis();
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Started calibration");
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
    case 8:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::Idle;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Motors off");
            break;
        }
        else
        {
            break;
        }
    }
    case 9:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::Calibration;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Started calibration");
            break;
        }
        else
        {
            break;
        }
    }
    case 10:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::LockedCurrent;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Motors locked in current position");
            break;
        }
        else
        {
            break;
        }
    }
    case 11:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::Debug;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Motors debug");
            break;
        }
        else
        {
            break;
        }
    }
    case 12:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::HotfireProfile;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Motors circle");
            break;
        }
        else
        {
            break;
        }
    }
    case 13:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::Armed;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Motors armed");
            break;
        }
        else
        {
            break;
        }
    }
    case 14:
    {
        if (currentEngineState == EngineState::Debug)
        {
            currOdriveState = oDriveState::pwmEnable;
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Enable PWM");
            break;
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

void NRCThanos::arm_impl(packetptr_t packetptr){

    if (m_calibrationDone){
        currOdriveState = oDriveState::Armed;
    }

    NRCRemoteActuatorBase::arm_impl(std::move(packetptr));

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

void NRCThanos::gotoChamberP(float target){
    m_oxServoCurrAngle = static_cast<float>(getOxAngle());
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("cp " + std::to_string(_chamberP));
    if ((_chamberP < target)  && m_oxServoCurrAngle < oxNominalAngle){
        gotoWithSpeed(oxServo, oxNominalAngle, m_firstNominalSpeed, m_oxServoPrevAngle, m_oxServoCurrAngle, m_oxServoPrevUpdate);
    }
    else if ((_chamberP < target) && (millis() - m_latestAngleUpdate > m_edgingDelay))
    {
        m_oxServoDemandAngle = m_oxServoCurrAngle + 5.0;
        if (m_oxServoDemandAngle >= oxMaxOpen){
            m_oxServoDemandAngle = oxMaxOpen;
        }
        oxServo.goto_Angle(m_oxServoDemandAngle);
        m_latestAngleUpdate = millis();
    }
    else if (_chamberP >= target)
    {
        //oxServo.goto_AngleHighRes(m_oxServoCurrAngle);
        oxServo.goto_Angle(m_oxServoCurrAngle);
    }

    m_oxPercent = (float)(m_oxServoCurrAngle - (float)oxServoPreAngle) / (float)(m_oxThrottleRange);
    m_fuelPercent = m_oxPercent + m_fuelExtra;
    float fuelAngle = (float)(m_fuelPercent * (float)m_fuelThrottleRange) + (float)fuelServoPreAngle;

    if (fuelAngle < (float)fuelServoPreAngle)
    {
        fuelServo.goto_AngleHighRes((float)fuelServoPreAngle);
    }
    else
    {
        fuelServo.goto_AngleHighRes(fuelAngle);
    }
}

void NRCThanos::gotoFuelP(float target){
    m_oxServoCurrAngle = static_cast<float>(getOxAngle());

    if ((_fuelP < target) && (millis() - m_latestAngleUpdate > m_edgingDelay))
    {
        m_oxServoDemandAngle = m_oxServoCurrAngle + 5.0;
        oxServo.goto_AngleHighRes(m_oxServoDemandAngle);
        m_latestAngleUpdate = millis();
    }
    else if (_fuelP > target)
    {
        oxServo.goto_AngleHighRes(m_oxServoCurrAngle);
    }

    m_oxPercent = (float)(m_oxServoCurrAngle - (float)oxServoPreAngle) / (float)(m_oxThrottleRange);
    m_fuelPercent = m_oxPercent + m_fuelExtra;
    float fuelAngle = (float)(m_fuelPercent * (float)m_fuelThrottleRange) + (float)fuelServoPreAngle;

    if (fuelAngle < (float)fuelServoPreAngle)
    {
        fuelServo.goto_AngleHighRes((float)fuelServoPreAngle);
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

void NRCThanos::openOxFill()
{
    SimpleCommandPacket openOxFillCmd(2, 180);
    openOxFillCmd.header.source_service = static_cast<uint8_t>(Services::ID::Thanos);
    openOxFillCmd.header.destination_service = m_oxFillService;
    openOxFillCmd.header.source = _address;
    openOxFillCmd.header.destination = m_oxFillNode;
    openOxFillCmd.header.uid = 0;
    _networkmanager.sendPacket(openOxFillCmd);
    oxFillClosed = false;
}

void NRCThanos::closeOxFill()
{
    if (!oxFillClosed){
        SimpleCommandPacket closeOxFillCmd(2, 0);
        closeOxFillCmd.header.source_service = static_cast<uint8_t>(Services::ID::Thanos);
        closeOxFillCmd.header.destination_service = m_oxFillService;
        closeOxFillCmd.header.source = _address;
        closeOxFillCmd.header.destination = m_oxFillNode;
        closeOxFillCmd.header.uid = 0;
        _networkmanager.sendPacket(closeOxFillCmd);

        if (closeOxFillCalls >= 2){     // this is clapped i know
            oxFillClosed = true;
            closeOxFillCalls = 0;
        }
        else{
            closeOxFillCalls++;
        }
    }
}


bool NRCThanos::pValUpdated()
{
    if ((millis() - lastTimeChamberPUpdate) > pressureUpdateTimeLim || (millis() - lastTimeThrustUpdate) > pressureUpdateTimeLim || (millis() - lastTimeFuelPUpdate) > pressureUpdateTimeLim)
    {
        return false;
    }
    else
    {
        return true;
    }
}