#pragma once

#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>



class NRCThanos : public NRCRemoteActuatorBase<NRCThanos>
{

    public:

        NRCThanos(RnpNetworkManager &networkmanager,
                    uint8_t fuelServoGPIO,
                    uint8_t fuelServoChannel,
                    uint8_t oxServoGPIO,
                    uint8_t oxServoChannel,
                    uint8_t address
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _networkmanager(networkmanager),      
            _fuelServoGPIO(fuelServoGPIO),
            _fuelServoChannel(fuelServoChannel),
            _oxServoGPIO(oxServoGPIO),
            _oxServoChannel(oxServoChannel),
            _address(address),
            fuelServo(fuelServoGPIO,fuelServoChannel,networkmanager),
            oxServo(oxServoGPIO,oxServoChannel,networkmanager)
            {};
        

        void setup();
        void update();
        void updateFuelP(float fuelP);
        void updateChamberP(float chamberP);
        

    protected:

        RnpNetworkManager& _networkmanager;
        const uint8_t _fuelServoGPIO;
        const uint8_t _fuelServoChannel;
        const uint8_t _oxServoGPIO;
        const uint8_t _oxServoChannel;
        const uint8_t _address;

        NRCRemoteServo fuelServo;
        NRCRemoteServo oxServo;    

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        
        void execute_impl(packetptr_t packetptr);
        //void arm_impl(packetptr_t packetptr);
        //void disarm_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr);

        enum class EngineState : uint8_t
        {
            Ignition = 0,
            ShutDown = 1,
            EngineController = 2,
            Default = 3,
            Override = 4
        };

        bool default_called = false;
        bool shutdown_called = false;

        EngineState currentEngineState = EngineState::Override;

        uint64_t ignitionTime;

        bool ignitionStarted = false;

        float _chamberP;
        float _fuelP;

        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalEngineOp();
        bool pValUpdated();
        float demandedFuelP();
        
        void firePyro(uint32_t duration);


        //Ignition sequence timings from moment ignition command received
        const uint64_t pyroFires = 0;
        const uint64_t fuelValvePreposition = 500;
        const uint64_t oxValvePreposition = 550;
        const uint64_t fuelValveFullBore = 1300;
        const uint64_t oxValveFullBore = 1350;
        const uint64_t endOfIgnitionSeq = 2100;

        float error;
        const float Kp = 2.5;
        uint16_t currFuelServoAngle;
        uint16_t fuelServoDemandAngle;
        const uint16_t fuelServoPreAngle = 60;
        const uint16_t oxServoPreAngle = 90;

        uint64_t lastTimeFuelPUpdate;
        uint64_t lastTimeChamberPUpdate;
        const uint64_t pressureUpdateTimeLim = 1000;

        uint64_t prevLogMessageTime;
};