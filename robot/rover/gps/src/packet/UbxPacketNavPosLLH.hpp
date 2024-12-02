#pragma once

#include "UbxPacket.hpp"

/**
 * @class UbxPacketNavPosLLH
 * @brief Represents a UBX NAV-POSLLH packet containing GPS position and accuracy data.
 *
 * This class handles the NAV-POSLLH message, which includes GPS time of week, latitude, longitude,
 * height (above ellipsoid and sea level), and accuracy estimates.
 *
 * Two constructors are provided: one for deserializing a packet from a frame, and one for serializing
 * a polling request with default values.
 *
 * @see UbxPacket for common packet functionality.
 */
class UbxPacketNavPosLLH : public UbxPacket {
 private:
	uint32_t iTOW{};        // GPS time of week of the navigation epoch.
	int32_t longitude{};
	int32_t latitude{};
	int32_t height_mm{};     // Height above ellipsoid in millimeters
	int32_t hMSL_mm{};       // Height above mean sea level in millimeters
	uint32_t hAcc_mm{};      // Horizontal accuracy estimate in millimeters
	uint32_t vAcc_mm{};      // Vertical accuracy estimate

 public:
	static const uint16_t DEFAULT_ID = 0x0201;
	const bool isPollingPacket;

	/**
 * @brief Constructs a UbxPacketNavPosLLH by deserializing data from a frame.
 *
 * This constructor initializes a UbxPacketNavPosLLH object by parsing a serialized
 * navigation frame. It ensures the packet has the correct PID and extracts fields
 * specific to the NAV-POSLLH UBX message type, such as time, position, height,
 * and accuracy values.
 *
 * @param frame A vector of bytes representing the serialized packet data.
 * @throws std::runtime_error If the PID in the frame does not match the expected PID.
 */
	explicit UbxPacketNavPosLLH(const std::vector<uint8_t>& frame);

	/**
 * @brief Constructs a UbxPacketNavPosLLH packet for polling purposes.
 *
 * This serializing constructor initializes the packet with the default PID for NAV-POSLLH
 * and sets the payload to an empty vector. The `isPollingPacket` flag is set to `true`.
 */
	UbxPacketNavPosLLH();

	// Getters for each member
	uint32_t getTimeOfWeek() const;

	int32_t getLongitude() const;

	int32_t getLatitude() const;

	int32_t getHeightAboveEllipsoid_mm() const;

	int32_t getHeightAboveSeaLevel_mm() const;

	uint32_t getHorizontalAccuracy_mm() const;

	uint32_t getVerticalAccuracy_mm() const;


};