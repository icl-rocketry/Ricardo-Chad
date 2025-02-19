/**
 * @file tvcTelemPacket.cpp
 * @author Riley Horrix (rh1122@ic.ac.uk)
 * @brief TVC Telemetry Packet Implementation
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "tvcTelemPacket.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include <vector>

const int TVC_PACKET_ID = 109;

TVCTelemPacket::~TVCTelemPacket() {};

TVCTelemPacket::TVCTelemPacket(): RnpPacket(0, TVC_PACKET_ID, size()) {};

TVCTelemPacket::TVCTelemPacket(const RnpPacketSerialized& packet): RnpPacket(packet,size()) {
    getSerializer().deserialize(*this,packet.getBody());
};

void TVCTelemPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};