#include "GPSController.hpp"

#include "packet/UbxPacketNavPosLLH.hpp"
#include "packet/UbxPacketNavSat.hpp"

GPSController::GPSController(const std::string& devicePath, uint8_t devaddr) :
	ubxConnection(std::make_unique<UbxI2CConnection>(devicePath, devaddr))
{
}

unique_ptr<UbxPacketNavPosLLH> GPSController::getGpsPosData() const
{
	auto posPacket = std::make_unique<UbxPacketNavPosLLH>();
	if (!ubxConnection->recvPacket(*posPacket))
	{
		std::cerr << "Failed to receive GPS data packet." << std::endl;
		return nullptr;
	}
	return posPacket;
}

unique_ptr<UbxPacketNavSat> GPSController::getSatelliteData() const
{
	auto satData = std::make_unique<UbxPacketNavSat>();
	if (!ubxConnection->recvPacket(*satData))
	{
		std::cerr << "Failed to receive Satellite data packet." << std::endl;
		return nullptr;
	}
	return satData;
}
