#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

//shamelessly copied from pickle rick's telemetry packet

class TMCTelemPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &TMCTelemPacket::armed,
                &TMCTelemPacket::status,
                &TMCTelemPacket::system_time
            );

            return ret;
        }

    public:
        ~TMCTelemPacket();

        TMCTelemPacket();
        /**
         * @brief Deserialize Telemetry Packet
         *
         * @param data
         */
        TMCTelemPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         *
         * @param buf
         */
        void serialize(std::vector<uint8_t>& buf) override;

        uint32_t status;
        bool armed;
        uint64_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


