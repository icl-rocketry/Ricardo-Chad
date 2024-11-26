// #pragma once 

// #include "librrc/nrcremoteactuatorbase.h"
// #include "librrc/nrcremoteservo.h"

// #include <librnp/rnp_networkmanager.h>
// #include <librnp/rnp_packet.h>

// #include "nrcremoteprop.h"


// class GNC : public NRCRemoteActuatorBase<GNC>
// {
//     public:
//         GNC(RnpNetworkManager &networkmanager,
//                     uint8_t topPropGPIO,
//                     uint8_t topPropChannel,
//                     uint8_t bottomPropGPIO,
//                     uint8_t bottomPropChannel,
//                     uint8_t address
//                     ):
//             NRCRemoteActuatorBase(networkmanager),
//             _networkmanager(networkmanager),      
//             _topPropGPIO(topPropGPIO),
//             _topPropChannel(topPropChannel),
//             _bottomPropGPIO(bottomPropGPIO),
//             _bottomPropChannel(bottomPropChannel),
//             _address(address),
//             topProp(topPropGPIO,topPropChannel,networkmanager),
//             bottomProp(bottomPropGPIO,bottomPropChannel,networkmanager)
//             {};
//         void setup();
//         void update();

//     protected:


//         RnpNetworkManager& _networkmanager;
//         const uint8_t _topPropGPIO;
//         const uint8_t _topPropChannel;
//         const uint8_t _bottomPropGPIO;
//         const uint8_t _bottomPropChannel;
//         const uint8_t _address;

//         NRCRemoteProp topProp;
//         NRCRemoteProp bottomProp;    

//         friend class NRCRemoteActuatorBase;
//         friend class NRCRemoteBase;

//         void execute_impl(packetptr_t packetptr);
//         //void arm_impl(packetptr_t packetptr);
//         //void disarm_impl(packetptr_t packetptr);
//         void override_impl(packetptr_t packetptr);
//         void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr);

//         enum class GNCState : uint8_t
//             {
//                 Idle = 0,
//                 Armed = 1,
//                 Abort = 2
            
//             };

//         GNCState currentGNCState = GNCState::Idle;

    
// };
