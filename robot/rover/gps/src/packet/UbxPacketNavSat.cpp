#include <cstdint>
#include <vector>

#include "UbxPacket.hpp"
#include "../serdes/UbxDeserializer.hpp"
#include "SatelliteInfo.hpp"

#include "UbxPacketNavSat.hpp"

uint16_t UbxPacketNavSat::PID = 0x3501;

void UbxPacketNavSat::ensurePid(uint16_t pid) const
{
	if (pid != PID) {
//		throw std::runtime_error("Invalid PID");
	}
}

// Deserializing constructor
UbxPacketNavSat::UbxPacketNavSat(const std::vector<uint8_t>& frame)
	: UbxPacket(frame), isPollingPacket{false}
{
	this->ensurePid(PID);
	UbxDeserializer des{this->payload};
	this->iTOW = des.readU4();
	this->version = des.readU1();
	this->numSvs = des.readU1();
	des.skip2Bytes();
	this->satellites.reserve(this->numSvs);
	for (int i = 0; i < this->numSvs; i++) {
		SatelliteInfo info{des};
		this->satellites.push_back(info);
	}
}

// Serializing constructor for polling
UbxPacketNavSat::UbxPacketNavSat()
	: UbxPacket(PID, {}), isPollingPacket{true}
{
}

// Getters
int UbxPacketNavSat::getNumSatellites() const
{
	return this->numSvs;
}

SatelliteInfo UbxPacketNavSat::getSatellite(int index) const
{
	return this->satellites.at(index);
}
