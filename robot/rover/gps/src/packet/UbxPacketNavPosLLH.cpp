#include "UbxPacketNavPosLLH.hpp"

#include <stdexcept>

#include "serdes/UbxDeserializer.hpp"

//	TODO omar: use DEFAULT_ID instead of PID here?
// Serializing constructor for polling
UbxPacketNavPosLLH::UbxPacketNavPosLLH()
	: UbxPacket(DEFAULT_ID, {}), isPollingPacket{ true }
{
}


UbxPacketNavPosLLH::UbxPacketNavPosLLH(const std::vector<uint8_t>& frame)
	: UbxPacket(frame), isPollingPacket{ false }
{
	UbxPacket::ensurePid(pid);
	UbxDeserializer des{ this->payload };
	this->iTOW = des.readU4();
	this->longitude = des.readI4();
	this->latitude = des.readI4();
	this->height_mm = des.readI4();
	this->hMSL_mm = des.readI4();
	this->hAcc_mm = des.readI4();
	this->vAcc_mm = des.readI4();
}

uint32_t UbxPacketNavPosLLH::getTimeOfWeek() const
{
	return iTOW;
}

int32_t UbxPacketNavPosLLH::getLongitude() const
{
	return longitude;
}

int32_t UbxPacketNavPosLLH::getLatitude() const
{
	return latitude;
}

int32_t UbxPacketNavPosLLH::getHeightAboveEllipsoid_mm() const
{
	return height_mm;
}

int32_t UbxPacketNavPosLLH::getHeightAboveSeaLevel_mm() const
{
	return hMSL_mm;
}

uint32_t UbxPacketNavPosLLH::getHorizontalAccuracy_mm() const
{
	return hAcc_mm;
}

uint32_t UbxPacketNavPosLLH::getVerticalAccuracy_mm() const
{
	return vAcc_mm;
}


void UbxPacketNavPosLLH::ensurePid(uint16_t pid) const
{
	if (pid != this->pid)
	{
		throw std::invalid_argument(
			"Invalid PID for UbxPacketNavSat. Expected: " + std::to_string(this->pid)
			+ ", Got: " + std::to_string(pid));
	}
}
