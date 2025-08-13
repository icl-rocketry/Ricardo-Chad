#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>


class RTKTelemetryPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &RTKTelemetryPacket::x_input,
                &RTKTelemetryPacket::y_input,
                &RTKTelemetryPacket::z_input,
                &RTKTelemetryPacket::u_input,
                &RTKTelemetryPacket::v_input,
                &RTKTelemetryPacket::w_input
            );

            return ret;
        }
        
    public:
        ~RTKTelemetryPacket();

        RTKTelemetryPacket();

        RTKTelemetryPacket(const RnpPacketSerialized& packet);

        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        float x_input;
        float y_input;
        float z_input;
        float u_input;
        float v_input;
        float w_input;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};