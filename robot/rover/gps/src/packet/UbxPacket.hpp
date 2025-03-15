#pragma once

#include <vector>
#include <cstdint>

class UbxPacket
{
 protected:
	uint16_t preamble;
	uint16_t pid;
	std::vector<uint8_t> payload;
	uint16_t chksum;
	std::vector<uint8_t> serializedFrame;

	uint16_t calculateChksum();

 public:
	virtual ~UbxPacket() = default;

	/**
	 * @brief Constructs a UbxPacket with the specified PID and payload.
	 *
	 * This serializing constructor is used to create a UbxPacket for sending data. It initializes
	 * the packet preamble, calculates the checksum, and serializes the packet into a
	 * frame ready for transmission.
	 *
	 * @param pid The packet identifier (PID) for this UBX packet.
	 * @param payload The payload data to include in the packet.
	 */
	UbxPacket(uint16_t pid, const std::vector<uint8_t>& payload);

	/**
 * @brief Constructs a UbxPacket by deserializing data from a frame.
 *
 * This constructor initializes a UbxPacket object by parsing a serialized frame
 * and extracting its components: preamble, PID, payload, and checksum.
 * It also verifies that the checksum in the frame matches the calculated checksum.
 *
 * @param frame A vector of bytes representing the serialized packet data.
 * @throws std::runtime_error If the checksum in the frame does not match the calculated checksum.
 */
	explicit UbxPacket(const std::vector<uint8_t>& frame);


	// getter
	virtual std::vector<uint8_t> serialize() const;

	uint16_t getPid() const
	{
		return this->pid;
	}

	uint8_t getClass() const
	{
		return this->pid & 0xFF;
	}

	uint8_t getID() const
	{
		return this->pid >> 8;
	}

	std::vector<uint8_t> getPayload() const
	{
		return this->payload;
	}

	uint16_t getChksum() const
	{
		return this->chksum;
	}

	virtual void ensurePid(uint16_t pid) const = 0;
};
