#pragma once

#include <vector>
#include <cstdint>

class UbxPacket {
 protected:
	uint16_t preamble;
	uint16_t pid;
	std::vector<uint8_t> payload;
	uint16_t chksum;
	std::vector<uint8_t> serializedFrame;

	uint16_t calculateChksum();

 public:
	virtual ~UbxPacket() = default;

	// Serializing constructor
	UbxPacket(uint16_t pid, const std::vector<uint8_t>& payload);

	// Deserializing constructor
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
