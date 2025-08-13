#include "RTKTelemetryPacket.h"
#include <vector>



RTKTelemetryPacket::~RTKTelemetryPacket()
{};

RTKTelemetryPacket::RTKTelemetryPacket():
RnpPacket(0,
          108, // ask andrei about types
          size())
{};

RTKTelemetryPacket::RTKTelemetryPacket(const RnpPacketSerialized& packet):
RnpPacket(packet,size())
{
    getSerializer().deserialize(*this,packet.getBody());
};

void RTKTelemetryPacket::deserializeBody(std::vector<uint8_t>& buf){
    getSerializer().deserialize(*this, buf);
}

void RTKTelemetryPacket::serialize(std::vector<uint8_t>& buf){
    RnpPacket::serialize(buf);
	size_t bufsize = buf.size();
	buf.resize(bufsize + size());
	std::memcpy(buf.data() + bufsize,getSerializer().serialize(*this).data(),size());
};