/**
 * @file TVCTelemPacket.h
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief TVC Telem Packet Definition.
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

/**
 * @brief Telemetry Packet for TVC information.
 * 
 * Shamelessly copied from Pickle Rick's telemetry packet
 */
class TVCTelemPacket : public RnpPacket{
private:
    /**
     * @brief Get the Serializer object
     * 
     * @return RNP Serializer
     */
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &TVCTelemPacket::time,
            &TVCTelemPacket::vbusVoltage,
            &TVCTelemPacket::state,
            &TVCTelemPacket::axis0Requested,
            &TVCTelemPacket::axis0Turns,
            &TVCTelemPacket::axis0Velocity,
            &TVCTelemPacket::axis0Current,
            &TVCTelemPacket::axis1Requested,
            &TVCTelemPacket::axis1Turns,
            &TVCTelemPacket::axis1Velocity,
            &TVCTelemPacket::axis1Current
        );

        return ret;
    }
    
public:
    TVCTelemPacket();
    
    ~TVCTelemPacket();
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

    float axis0Requested;
    float axis1Requested;
    uint32_t time;
    float vbusVoltage;
    uint32_t state;
    float axis0Turns;
    float axis0Velocity;
    float axis0Current;
    float axis1Turns;
    float axis1Velocity;
    float axis1Current;

    static constexpr size_t size(){
        return getSerializer().member_size();
    }
};
