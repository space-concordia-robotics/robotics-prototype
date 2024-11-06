#include "UbxSerializer.hpp"

UbxSerializer::UbxSerializer()
{
}

// Pre-allocate some space for the vector
UbxSerializer::UbxSerializer(uint32_t initialSize)
{
    this->payload.reserve(initialSize);
}

// Write unsigned 8-bit integer
UbxSerializer& UbxSerializer::writeU1(uint8_t data) // 1 -> [1] , -1 -> [1]
{
    this->payload.push_back(data);
    return *this;
}

// Write two's complement 8-bit integer
UbxSerializer& UbxSerializer::writeI1(int8_t data) // -1 -> 255
{
    this->writeU1(static_cast<uint8_t>(data));
    return *this;
}

// Write unsigned 16-bit integer
UbxSerializer& UbxSerializer::writeU2(uint16_t data)
{
    this->payload.push_back(static_cast<uint8_t>(data & 0xFF));
    this->payload.push_back(static_cast<uint8_t>(data >> 8 & 0xFF));
    return *this;
}

// Write two's complement 16-bit integer
UbxSerializer& UbxSerializer::writeI2(int16_t data)
{
    this->writeU2(static_cast<uint16_t>(data));
    return *this;
}

// Write unsigned 32-bit integer
UbxSerializer& UbxSerializer::writeU4(uint32_t data)
{
    this->payload.push_back(static_cast<uint8_t>(data & 0xFF));
    this->payload.push_back(static_cast<uint8_t>(data >> 8 & 0xFF));
    this->payload.push_back(static_cast<uint8_t>(data >> 16 & 0xFF));
    this->payload.push_back(static_cast<uint8_t>(data >> 24 & 0xFF));
    return *this;
}

// Write two's complement 32-bit integer
UbxSerializer& UbxSerializer::writeI4(int32_t data)
{
    this->writeU4(static_cast<uint32_t>(data));
    return *this;
}

// Write multiple bytes
UbxSerializer& UbxSerializer::writeByteArray(const uint8_t* data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        this->payload.push_back(data[i]);
    }
    return *this;
}

// Write std::vector
UbxSerializer& UbxSerializer::writeVector(const std::vector<uint8_t>& data)
{
    return this->writeByteArray(data.data(), data.size());
}

// Write IEEE 32-bit floating point number
UbxSerializer& UbxSerializer::writeR4(float data)
{
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);
    this->writeByteArray(ptr, 4);
    return *this;
}

// Write IEEE 64-bit floating point number
UbxSerializer& UbxSerializer::writeR8(double data)
{
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);
    this->writeByteArray(ptr, 8);
    return *this;
}

// Write character
UbxSerializer& UbxSerializer::writeCh(char data)
{
    this->payload.push_back(static_cast<uint8_t>(data));
    return *this;
}

// Write one bit to bitfield buffer
UbxSerializer& UbxSerializer::writeXF(int bit, bool data)
{
    if (bit < 0 || bit > 31)
    {
        return *this;
    }
    if (data)
    {
        this->bitfieldBuffer |= 0x1 << bit;
    }
    else
    {
        this->bitfieldBuffer &= ~(0x1 << bit);
    }
    return *this;
}

// Write unsigned integer to bitfield buffer
UbxSerializer& UbxSerializer::writeXU(int msb, int lsb, uint32_t data)
{
    if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb < lsb)
    {
        return *this;
    }
    uint32_t clear = (msb < 31 ? (1 << msb + 1) : 0) - 1 << lsb;
    uint32_t set = data << lsb;
    this->bitfieldBuffer = this->bitfieldBuffer & clear | set;
    return *this;
}

// Write two's complement integer to bitfield buffer
UbxSerializer& UbxSerializer::writeXI(int msb, int lsb, int32_t data)
{
    this->writeXU(msb, lsb, static_cast<uint32_t>(data));
    return *this;
}

// Write sign bit + magnitude represented integer to bitfield buffer
UbxSerializer& UbxSerializer::writeXS(int msb, int lsb, int32_t data)
{
    if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb <= lsb) // Note the <=, we need at least 2 bits
    {
        return *this;
    }
    // Write sign bit
    this->writeXF(msb, data < 0);
    // Write magnitude
    uint32_t magnitude = data >= 0 ? data : -data;
    this->writeXU(msb - 1, lsb, magnitude);
    return *this;
}

// Write 8-bit bitfield to payload
UbxSerializer& UbxSerializer::writeX1()
{
    this->writeU1(static_cast<uint8_t>(this->bitfieldBuffer));
    this->bitfieldBuffer = 0;
    return *this;
}

// Write 16-bit bitfield to payload
UbxSerializer& UbxSerializer::writeX2()
{
    this->writeU2(static_cast<uint8_t>(this->bitfieldBuffer));
    this->bitfieldBuffer = 0;
    return *this;
}

// Write 32-bit bitfield to payload
UbxSerializer& UbxSerializer::writeX4()
{
    this->writeU4(static_cast<uint8_t>(this->bitfieldBuffer));
    this->bitfieldBuffer = 0;
    return *this;
}

const std::vector<uint8_t>& UbxSerializer::getPayloadReadOnly()
{
    return this->payload;
}
