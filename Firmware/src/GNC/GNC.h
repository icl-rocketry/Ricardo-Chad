#pragma once 

#include "prop.h"
#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>


class GNC : public NRCRemoteActuatorBase<GNC>
{
    public:
        GNC(RnpNetworkManager &networkmanager,
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
            externalServo(fuelServoGPIO,fuelServoChannel,networkmanager,0,0,180,0,170),
            internalServo(oxServoGPIO,oxServoChannel,networkmanager,0,0,180,10,170)
            {};
        void setup();
        void update();

    protected:


        RnpNetworkManager& _networkmanager;
        const uint8_t _fuelServoGPIO;
        const uint8_t _fuelServoChannel;
        const uint8_t _oxServoGPIO;
        const uint8_t _oxServoChannel;
        const uint8_t _address;

        NRCRemoteServo externalServo;
        NRCRemoteServo internalServo;    

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        void execute_impl(packetptr_t packetptr);
        //void arm_impl(packetptr_t packetptr);
        //void disarm_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr);

        enum class GNCState : uint8_t
            {
                Idle = 0,
                Arm = 1,
                Abort = 2
            
            };

        GNCState currentGNCState = GNCState::Idle;

    
};
