#include "ChadTelemPacket.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include <vector>



ChadTelemPacket::~ChadTelemPacket()
{};

ChadTelemPacket::ChadTelemPacket():
RnpPacket(0,
          104,
          size())
{};

ChadTelemPacket::ChadTelemPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void ChadTelemPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};