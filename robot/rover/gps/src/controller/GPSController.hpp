#pragma once

#include <algorithm>
#include <memory>

#include "connection/UbxI2CConnection.hpp"
#include "packet/UbxPacketNavPosLLH.hpp"
#include "packet/UbxPacketNavSat.hpp"

using std::unique_ptr;

/**
 * @brief Handles GPS communication via UBX over I2C.
 *
 * @throws std::runtime_error if the I2C connection fails.
 */
class GPSController
{
 private:
	unique_ptr<UbxAbstractConnection> ubxConnection;

 public:
	/**
 * @brief Constructs a GPSController and initializes the I2C connection.
 *
 * @param devicePath The I2C device path (e.g., "/dev/i2c-1").
 * @param devaddr The I2C address of the GPS module.
 * @throws std::runtime_error if the I2C connection cannot be established.
 */
	GPSController(const std::string& devicePath, uint8_t devaddr);

	unique_ptr<UbxPacketNavPosLLH> getGpsPosData() const;

	unique_ptr<UbxPacketNavSat> getSatelliteData() const;
};
