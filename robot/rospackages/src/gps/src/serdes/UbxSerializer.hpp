#pragma once

#include <cstdint>
#include <vector>

class UbxSerializer
{
private:
    std::vector<uint8_t> payload;
    uint32_t bitfieldBuffer = 0;

public:
    UbxSerializer();

    // Pre-allocate some space for the vector
    UbxSerializer(uint32_t initialSize);

    // Write unsigned 8-bit integer
    UbxSerializer& writeU1(uint8_t data);

    // Write two's complement 8-bit integer
    UbxSerializer& writeI1(int8_t data);

    // Write unsigned 16-bit integer
    UbxSerializer& writeU2(uint16_t data);

    // Write two's complement 16-bit integer
    UbxSerializer& writeI2(int16_t data);

    // Write unsigned 32-bit integer
    UbxSerializer& writeU4(uint32_t data);

    // Write two's complement 32-bit integer
    UbxSerializer& writeI4(int32_t data);

    // Write multiple bytes
    UbxSerializer& writeByteArray(const uint8_t* data, uint32_t length);

    // Write std::vector
    UbxSerializer& writeVector(const std::vector<uint8_t> & data); 

    // Write IEEE 32-bit floating point number
    UbxSerializer& writeR4(float data);

    // Write IEEE 64-bit floating point number
    UbxSerializer& writeR8(double data);

    // Write character
    UbxSerializer& writeCh(char data);

    // Write one bit to bitfield buffer
    UbxSerializer& writeXF(int bit, bool data);

    // Write unsigned integer to bitfield buffer
    UbxSerializer& writeXU(int msb, int lsb, uint32_t data);

    // Write two's complement integer to bitfield buffer
    UbxSerializer& writeXI(int msb, int lsb, int32_t data);

    // Write sign bit + magnitude represented integer to bitfield buffer
    UbxSerializer& writeXS(int msb, int lsb, int32_t data);

    // Write 8-bit bitfield to payload
    UbxSerializer& writeX1();

    // Write 16-bit bitfield to payload
    UbxSerializer& writeX2();

    // Write 32-bit bitfield to payload
    UbxSerializer& writeX4();

    const std::vector<uint8_t>& getPayloadReadOnly();
};
