#pragma once

#include <cstdint>
#include "../serdes/UbxDeserializer.hpp"


struct SatelliteInfo {
 public:
	const uint8_t gnssId; // GNSS ID
	const uint8_t svId; // Satellite ID
	const uint8_t cno; // Carrier to noise ratio
	const int8_t elev; // Elevation, in degrees
	const int16_t azim; // Azimuth, in degrees
	const int16_t prRes; // Pseudorange residual, in 0.1m
	const uint8_t qualityInd; // Quality indicator (TODO: implement as ENUM?)
	const bool svUsed; // Whether satellite is used
	const uint8_t health; // Health indicator (TODO: implement as ENUM?)
	const bool diffCorr;
	const bool smoothed;
	const uint8_t orbitSource; // (TODO: implement as ENUM?)
	const bool ephAvail;
	const bool almAvail;
	const bool anoAvail;
	const bool aopAvail;
	const bool sbasCorrUsed;
	const bool rtcmCorrUsed;
	const bool slasCorrUsed;
	const bool spartnCorrUsed;
	const bool prCorrUsed;
	const bool crCorrUsed;
	const bool doCorrUsed;
	const bool clasCorrUsed;
	/*
		uint8_t num   = reader.readXU(5, 4, uint8_t);
		uint32_t num2 = reader.readXU(31, 6, uint32_t); // 31 - 6 + 1
	*/


	// Deserializing constructor only
	SatelliteInfo(UbxDeserializer& reader) :
		gnssId{reader.readU1()},
		svId{reader.readU1()},
		cno{reader.readU1()},
		elev{reader.readI1()},
		azim{reader.readI2()},
		prRes{reader.readI2()},
		qualityInd{static_cast<uint8_t>((reader.readX4(), reader.readXU(2, 0)))},
		svUsed{reader.readXF(3)},
		health{static_cast<uint8_t>(reader.readXU(5, 4))},
		diffCorr{reader.readXF(6)},
		smoothed{reader.readXF(7)},
		orbitSource{static_cast<uint8_t>(reader.readXU(10, 8))},
		ephAvail{reader.readXF(11)},
		almAvail{reader.readXF(12)},
		anoAvail{reader.readXF(13)},
		aopAvail{reader.readXF(14)},
		sbasCorrUsed{reader.readXF(16)},
		rtcmCorrUsed{reader.readXF(17)},
		slasCorrUsed{reader.readXF(18)},
		spartnCorrUsed{reader.readXF(19)},
		prCorrUsed{reader.readXF(20)},
		crCorrUsed{reader.readXF(21)},
		doCorrUsed{reader.readXF(22)},
		clasCorrUsed{reader.readXF(23)}
	{
	}
};

