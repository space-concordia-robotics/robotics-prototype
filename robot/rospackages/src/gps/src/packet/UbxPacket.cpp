#include "UbxPacket.hpp"

#include "serdes/UbxSerializer.hpp"
#include "serdes/UbxDeserializer.hpp"

// Serializing Constructor
UbxPacket::UbxPacket(uint16_t pid, const std::vector<uint8_t> payload)
{
    this->preamble = 0x62B5;
    this->pid = pid;
    this->payload = payload; // Do I need to copy if I'm not passing a const &?
    this->chksum = this->calculateChksum();
    UbxSerializer ser(8 + payload.size());
    this->serializedFrame =
        ser.writeU2(this->preamble)
           .writeU2(this->pid)
           .writeU2(payload.size())
           .writeVector(payload)
           .writeU2(this->chksum)
           .getPayloadReadOnly();
}

// Deserializing Constructor
UbxPacket::UbxPacket(const std::vector<uint8_t> frame)
{
    UbxDeserializer des(frame);
    this->preamble = des.readU2();
    this->pid = des.readU2();
    this->payload = des.readVector(frame.size() - 8);
    this->chksum = des.readU2();
    des.assertEmpty();

    uint16_t checksum = this->calculateChksum();
    if (checksum != this->chksum)
    {
        // TODO: throw an exception
        // For now we can ignore it
    }

    this->serializedFrame = frame; // Do I need to copy if I'm not passing a const &?
}

UbxPacket::~UbxPacket()
{
}


uint16_t UbxPacket::calculateChksum()
{
    uint8_t a = 0, b = 0;
    int N = this->payload.size();
    const uint8_t header[] = {
        static_cast<uint8_t>(this->pid & 0xFF),
        static_cast<uint8_t>(this->pid >> 8),
        static_cast<uint8_t>(N & 0xFF),
        static_cast<uint8_t>(N >> 8)
    };

    for (int i = 0; i < sizeof(header); i++)
    {
        a += header[i];
        b += a;
    }

    for (int i = 0; i < N; i++)
    {
        a += this->payload.at(i);
        b += a;
    }

    return static_cast<uint16_t>(a) | (static_cast<uint16_t>(b) << 8);
}

std::vector<uint8_t> UbxPacket::serialize() const
{
    return this->serializedFrame;
}

void UbxPacket::ensurePid(uint16_t pid) const
{
    if(this->pid != pid)
    {
        // TODO: throw exception later
    }
}
