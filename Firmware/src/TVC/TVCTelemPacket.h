#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

//shamelessly copied from pickle rick's telemetry packet

class TVCTelemPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &TVCTelemPacket::odriveVoltage,
                &TVCTelemPacket::axis0Turns,
                &TVCTelemPacket::axis0Velocity,
                &TVCTelemPacket::axis1Turns,
                &TVCTelemPacket::axis1Velocity
            );

            return ret;
        }
        
    public:
        ~TVCTelemPacket();

        TVCTelemPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        TVCTelemPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        float odriveVoltage;
        float axis0Turns;
        float axis0Velocity;
        // float axis0Current;
        float axis1Turns;
        float axis1Velocity;
        // float axis1Current;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


