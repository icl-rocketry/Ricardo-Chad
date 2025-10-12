#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

//shamelessly copied from pickle rick's telemetry packet

class FTSChadTelemPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &FTSChadTelemPacket::system_time,
                &FTSChadTelemPacket::fts_deployed
            );

            return ret;
        }

    public:
        ~FTSChadTelemPacket();

        FTSChadTelemPacket();
        /**
         * @brief Deserialize Telemetry Packet
         *
         * @param data
         */
        FTSChadTelemPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         *
         * @param buf
         */
        void serialize(std::vector<uint8_t>& buf) override;

        bool fts_deployed;
        uint64_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


