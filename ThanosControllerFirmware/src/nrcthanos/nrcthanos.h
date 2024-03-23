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
            fuelServo(fuelServoGPIO,fuelServoChannel,networkmanager,0,0,180,0,170),
            oxServo(oxServoGPIO,oxServoChannel,networkmanager,0,0,180,10,170)
            {};
        

        void setup();
        void update();
        void updateFuelP(float fuelP);
        void updateChamberP(float chamberP);
        bool getPollingStatus(){return _polling;};
        
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
            Default = 0,
            Ignition = 1,
            ShutDown = 2,
            EngineController = 3,
            Fullbore = 4,
            Debug = 5
        };

        bool fullbore_called = false;
        bool shutdown_called = false;

        EngineState currentEngineState = EngineState::Default;

        uint64_t ignitionTime;

        float _chamberP;
        float _fuelP;

        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalEngineOp();
        bool pValUpdated();
        float demandedFuelP();
        
        void firePyro(uint32_t duration);


        //Ignition sequence timings from moment ignition command received
        const uint64_t pyroFires = 0;
        const uint64_t fuelValvePreposition = 5500;
        const uint64_t oxValvePreposition = 5000;
        const uint64_t fuelValveFullBore = 6800;
        const uint64_t oxValveFullBore = 6800;
        const uint64_t endOfIgnitionSeq = 7500;

        float error;
        const float Kp = 2.5;
        uint16_t currFuelServoAngle;
        uint16_t fuelServoDemandAngle;
        const uint16_t fuelServoPreAngle = 60;
        const uint16_t oxServoPreAngle = 55;

        uint64_t lastTimeFuelPUpdate;
        uint64_t lastTimeChamberPUpdate;

        const uint64_t pressureUpdateTimeLim = 1000;

        uint8_t _ignitionCalls = 0;
        const uint8_t _ignitionCommandMaxCalls = 2;
        const uint8_t _ignitionCommandSendDelta = 50;
        uint32_t _prevFiring = 0;

        bool _polling = false;
};