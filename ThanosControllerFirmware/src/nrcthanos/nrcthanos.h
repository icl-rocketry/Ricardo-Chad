#pragma once

#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include <SiC43x.h>


class NRCThanos : public NRCRemoteActuatorBase<NRCThanos>
{

    public:

        NRCThanos(RnpNetworkManager &networkmanager,
                    uint8_t fuelServoGPIO,
                    uint8_t fuelServoChannel,
                    uint8_t oxServoGPIO,
                    uint8_t oxServoChannel,
                    uint8_t overrideGPIO,
                    uint8_t tvc0,
                    uint8_t tvc1,
                    uint8_t tvc2,
                    uint8_t address,
                    SiC43x& Buck
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _networkmanager(networkmanager),      
            _fuelServoGPIO(fuelServoGPIO),
            _fuelServoChannel(fuelServoChannel),
            _oxServoGPIO(oxServoGPIO),
            _oxServoChannel(oxServoChannel),
            _overrideGPIO(overrideGPIO),
            _tvcpin0(tvc0),
            _tvcpin1(tvc1),
            _tvcpin2(tvc2),
            _address(address),
            fuelServo(fuelServoGPIO,fuelServoChannel,networkmanager,0,0,180,0,175),
            oxServo(oxServoGPIO,oxServoChannel,networkmanager,0,0,180,10,160),
            _Buck(Buck)
            {};

        void setup();
        void update();
        void updateThrust(float thrust);
        void thanosStateMachine();
        void odriveStateMachine();
        void updateChamberP(float chamberP);
        void updateFuelP(float fuelP);
        bool getPollingStatus() { return _polling; };

        uint16_t getFuelAngle() { return fuelServo.getAngle(); };
        uint16_t getOxAngle() { return oxServo.getAngle(); };
        uint8_t getStatus(){return static_cast<uint8_t>(currentEngineState);};
        uint8_t getoDrvStatus(){return static_cast<uint8_t>(currOdriveState);};

    protected:
        RnpNetworkManager &_networkmanager;
        const uint8_t _fuelServoGPIO;
        const uint8_t _fuelServoChannel;
        const uint8_t _oxServoGPIO;
        const uint8_t _oxServoChannel;
        const uint8_t _overrideGPIO;
        const uint8_t _tvcpin0;
        const uint8_t _tvcpin1;
        const uint8_t _tvcpin2;

        const uint8_t _address;

        NRCRemoteServo fuelServo;
        NRCRemoteServo oxServo;

        SiC43x& _Buck;

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        void execute_impl(packetptr_t packetptr);
        void arm_impl(packetptr_t packetptr);
        // void disarm_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);

        // enum class EngineState : uint8_t
        // {
        //     Default = 0,
        //     Ignition = 1,
        //     ShutDown = 2,
        //     NominalT = 3,
        //     ThrottledT = 4,
        //     // Fullbore = 4,
        //     Debug = 5
        // };

        enum class EngineState : uint8_t
        {
            Default = 1<<0,
            Ignition = 1<<1,
            ShutDown = 1<<2,
            NominalT = 1<<3,
            TVCCircle = 1<<4,
            Calibration = 1<<5,
            // Fullbore = 4,
            Debug = 1<<6
            //last states of this enum are odrive states
        };

        enum class oDriveState : uint8_t
        {
            Idle = 1<<0,
            Armed = 1<<1,
            LockedCurrent = 1<<2,
            HotfireProfile = 1<<3,
            Calibration = 1<<4,
            Debug = 1<<5
        };


        bool fullbore_called = false;
        bool shutdown_called = false;

        EngineState currentEngineState = EngineState::Default;
        oDriveState currOdriveState = oDriveState::Idle;

        uint64_t ignitionTime = 0;

        float _chamberP;
        float _thrust;
        float _fuelP;

        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalEngineOp();
        bool pValUpdated();

        void gotoWithSpeed(NRCRemoteServo &Servo, uint16_t demandAngle, float speed, float &prevAngle, float &currAngle, uint32_t &prevUpdateT);

        void gotoThrust(float target, float closespeed, float openspeed);
        void gotoChamberP(float target);
        void gotoFuelP(float target);
        void firePyro(uint32_t duration);
        void openOxFill();
        void closeOxFill();

        void resetVars(){
            m_fuelServoPrevUpdate = 0;
            m_oxServoPrevUpdate = 0;
            m_fuelServoPrevAngle = fuelServo.getValue();
            m_oxServoPrevAngle = oxServo.getValue();
            m_thrustreached = false;
            m_tvcEntered = false;
        };

        // Ignition sequence timings from moment ignition command received
        const uint64_t pyroFires = 0;
        const uint64_t fuelValveNominal = 500;
        const uint64_t oxValveNominal = 550;
        const uint64_t endOfIgnitionSeq = 1050;

        const float m_targetChamberP = 12.5;
        const float m_targetFuelP = 8;
        const float m_targetBuffer = 0.02;

        const uint16_t fuelServoPreAngle = 105;
        const uint16_t oxServoPreAngle = 70;

        const uint16_t fuelMaxOpen = 150;
        const uint16_t oxMaxOpen = 150;

        const uint16_t fuelNominalAngle = 122;
        const uint16_t oxNominalAngle = 122;

        uint64_t lastTimeThrustUpdate;
        uint64_t lastTimeChamberPUpdate;
        uint64_t lastTimeFuelPUpdate;
        uint64_t m_latestAngleUpdate;

        const uint64_t pressureUpdateTimeLim = 1000;
        const uint32_t m_firstNominalTime = 2000;
        const uint32_t m_tvctime = 2000;
        const uint32_t m_secondNominalTime = 3000;
        const uint32_t m_cutoffTime = 14000;
        const uint32_t m_calibrationTime = 65000;
        const uint32_t m_motorsLockTime = 1000;
        const uint32_t m_oxDelay = 100;
        const uint32_t m_startTVCCircle = 2000;
        const uint32_t m_oxFillCloseTime = 10500;
        const uint32_t m_edgingDelay = 300;

        uint8_t _ignitionCalls = 0;
        const uint8_t _ignitionCommandMaxCalls = 2;
        const uint8_t _ignitionCommandSendDelta = 50;
        uint32_t _prevFiring = 0;

        bool _polling = false;

        bool oxFillClosed = false;
        uint8_t closeOxFillCalls = 0;

        //
        const uint8_t m_ingitionService = 12;
        const uint8_t m_ignitionNode = 107;

        const uint8_t m_oxFillService = 10;
        const uint8_t m_oxFillNode = 103;

        float m_fuelServoCurrAngle = 0;
        float m_oxServoCurrAngle = 0;

        float m_fuelServoPrevAngle = 0;
        float m_oxServoPrevAngle = 0;

        float m_fuelServoDemandAngle = 0;
        float m_oxServoDemandAngle = 0;

        uint32_t m_fuelServoPrevUpdate = 0;
        uint32_t m_oxServoPrevUpdate = 0;

        const float m_servoFast = 75; // degs per second
        const float m_firstNominalSpeed = 120; // degs per second
        const float m_servoSlow = 20;  // degs per second

        //
        bool m_thrustreached = false;
        uint32_t m_throttledEntry;
        uint32_t m_nominalEntry;
        uint32_t m_tvcEntry;
        bool m_tvcEntered = false;
        uint32_t m_calibrationStart;
        bool m_firstNominal = false;

        bool m_calibrationDone = false;

        //
        float m_oxPercent = 0;
        float m_fuelPercent = 0;
        uint16_t m_oxThrottleRange = 0;
        uint16_t m_fuelThrottleRange = 0;

        float m_fuelExtra = -0.1;      //0.05 for hotfire

        void motorsOff(){
            digitalWrite(_tvcpin0,LOW);
            digitalWrite(_tvcpin1,LOW);
            digitalWrite(_tvcpin2,LOW);
        };

        void motorsCalibrate(){
            digitalWrite(_tvcpin0,HIGH);
            digitalWrite(_tvcpin1,LOW);
            digitalWrite(_tvcpin2,LOW);
        };

        void motorsLocked(){
            digitalWrite(_tvcpin0,LOW);
            digitalWrite(_tvcpin1,HIGH);
            digitalWrite(_tvcpin2,LOW);
        };

        void motorsCircle(){
            digitalWrite(_tvcpin0,HIGH);
            digitalWrite(_tvcpin1,HIGH);
            digitalWrite(_tvcpin2,LOW);
        };

        void motorsDebug(){
            digitalWrite(_tvcpin0,LOW);
            digitalWrite(_tvcpin1,LOW);
            digitalWrite(_tvcpin2,HIGH);
        };

        void motorsArmed(){
            digitalWrite(_tvcpin0,LOW);
            digitalWrite(_tvcpin1,HIGH);
            digitalWrite(_tvcpin2,HIGH);
        };

        
};