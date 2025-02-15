#include "TVCTelemPacket.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include <vector>

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