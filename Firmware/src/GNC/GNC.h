#pragma once 

#include "prop.h"
#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>


class GNC : public NRCRemoteActuatorBase<GNC>
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
            externalServo(fuelServoGPIO,fuelServoChannel,networkmanager,0,0,180,0,170),
            internalServo(oxServoGPIO,oxServoChannel,networkmanager,0,0,180,10,170)
            {};
    protected:
        enum class GNCState : uint8_t
            {
                Idle = 0,
                Arm = 1,
                Abort = 2,
            
            };
        GNCState currentGNCState = GNCState::Idle;
}
