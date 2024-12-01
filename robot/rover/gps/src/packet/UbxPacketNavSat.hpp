#pragma once

#include <vector>
#include <cstdint>
#include "SatelliteInfo.hpp"
#include "UbxPacket.hpp"
#include "../serdes/UbxDeserializer.hpp"

// UbxPacketNavSat: Handles satellite navigation data
class UbxPacketNavSat : public UbxPacket {
 private:
	uint32_t iTOW{};
	uint8_t version{};
	uint8_t numSvs{};
	std::vector<SatelliteInfo> satellites;

 public:
	static uint16_t PID;
	const bool isPollingPacket;

	// Ensures the packet ID matches
	void ensurePid(uint16_t pid) const override;

	// Deserializing constructor
	explicit UbxPacketNavSat(const std::vector<uint8_t>& frame);

	// Serializing constructor for polling
	UbxPacketNavSat();

	// Getters
	int getNumSatellites() const;

	SatelliteInfo getSatellite(int index) const;
};

