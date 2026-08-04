#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

//shamelessly copied from pickle rick's telemetry packet

class ChadPotsPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &ChadPotsPacket::Pot0Percentage,
                &ChadPotsPacket::Pot1Percentage,
                &ChadPotsPacket::Pot2Percentage,
                &ChadPotsPacket::Pot3Percentage
            );

            return ret;
        }
        
    public:
        ~ChadPotsPacket();

        ChadPotsPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        ChadPotsPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        int Pot0Percentage = 0;
        int Pot1Percentage = 0;
        int Pot2Percentage = 0;
        int Pot3Percentage = 0;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


