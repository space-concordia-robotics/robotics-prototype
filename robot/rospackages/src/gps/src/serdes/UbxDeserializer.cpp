#include "UbxDeserializer.hpp"


uint8_t UbxDeserializer::readByteNochk()
{
    return this->payload.at(this->index++);
}

UbxDeserializer::UbxDeserializer(std::vector<uint8_t> payload)
{
    this->payload = payload;
    this->index = 0;
}

void UbxDeserializer::reset()
{
    this->index = 0;
}

void UbxDeserializer::assertBytesAvailable(uint32_t length)
{
    uint32_t requiredLength = this->index + length;
    if (this->payload.size() < requiredLength)
    {
        throw UbxPayloadDrainedException(this->payload, this->index, length);
    }
}

void UbxDeserializer::skipBytes(int bytesToSkip) noexcept(false)
{
    assertBytesAvailable(bytesToSkip);
    index += bytesToSkip;
}

void UbxDeserializer::skip1Byte()
{
    skipBytes(1);
}

void UbxDeserializer::skip2Bytes()
{
    skipBytes(2);
}

void UbxDeserializer::skip4Bytes()
{
    skipBytes(4);
}


void UbxDeserializer::assertEmpty()
{
    if (this->index < this->payload.size())
    {
        // TODO: josh readLength here?
        throw UbxPayloadExcessException(this->payload, this->index, 0);
    }
}

uint8_t UbxDeserializer::readU1()
{
    this->assertBytesAvailable(1);
    return this->readByteNochk();
}

int8_t UbxDeserializer::readI1()
{
    this->assertBytesAvailable(1);
    return static_cast<int8_t>(this->readByteNochk());
}

uint16_t UbxDeserializer::readU2()
{
    this->assertBytesAvailable(2);
    uint16_t data;
    data = static_cast<uint16_t>(this->readByteNochk());
    data |= (static_cast<uint16_t>(this->readByteNochk())) << 8;
    return data;
}

int16_t UbxDeserializer::readI2()
{
    return static_cast<int16_t>(this->readU2());
}

uint32_t UbxDeserializer::readU4()
{
    this->assertBytesAvailable(4);
    uint32_t data;
    data = static_cast<uint32_t>(this->readByteNochk());
    data |= (static_cast<uint32_t>(this->readByteNochk())) << 8;
    data |= (static_cast<uint32_t>(this->readByteNochk())) << 16;
    data |= (static_cast<uint32_t>(this->readByteNochk())) << 24;
    return data;
}

int32_t UbxDeserializer::readI4()
{
    return static_cast<int32_t>(this->readU4());
}

void UbxDeserializer::readByteArray(uint8_t* data, uint32_t length)
{
    this->assertBytesAvailable(length);
    for (int i = 0; i < length; i++)
    {
        data[i] = this->readByteNochk();
    }
}

std::vector<uint8_t> UbxDeserializer::readVector(uint32_t length)
{
    this->assertBytesAvailable(length);
    std::vector<uint8_t> data;
    data.resize(length);
    this->readByteArray(data.data(), data.size());
    return data;
}

float UbxDeserializer::readR4()
{
    float data;
    this->readByteArray(reinterpret_cast<uint8_t*>(&data), 4);
    return data;
}

double UbxDeserializer::readR8()
{
    double data;
    this->readByteArray(reinterpret_cast<uint8_t*>(&data), 8);
    return data;
}

char UbxDeserializer::readCh()
{
    this->assertBytesAvailable(1);
    return static_cast<char>(this->readByteNochk());
}

void UbxDeserializer::readX1()
{
    this->bitfieldBuffer = static_cast<uint32_t>(this->readU1());
}

void UbxDeserializer::readX2()
{
    this->bitfieldBuffer = static_cast<uint32_t>(this->readU2());
}

void UbxDeserializer::readX4()
{
    this->bitfieldBuffer = this->readU4();
}

bool UbxDeserializer::readXF(int bit)
{
    if (bit < 0 || bit > 31)
    {
        return false;
    }
    return !!(this->bitfieldBuffer & (1 << bit));
}

uint32_t UbxDeserializer::readXU(int msb, int lsb)
{
    if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb < lsb)
    {
        return 0;
    }
    uint32_t temp = this->bitfieldBuffer >> lsb;
    if (msb < 31)
    {
        uint32_t mask = (1 << msb - lsb + 1) - 1;
        return temp & mask;
    }
    else
    {
        return temp;
    }
}

int32_t UbxDeserializer::readXI(int msb, int lsb)
{
    if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb < lsb)
    {
        return 0;
    }
    int32_t temp = static_cast<int32_t>(this->bitfieldBuffer) >> lsb;
    if (msb < 31)
    {
        uint32_t mask = (1 << msb - lsb + 1) - 1;
        return temp & mask;
    }
    else
    {
        return temp;
    }
}

int32_t UbxDeserializer::readXS(int msb, int lsb)
{
    if (msb < 0 || lsb < 0 || msb > 31 || lsb > 31 || msb <= lsb) // Note the <=, we need at least 2 bits
    {
        return 0;
    }
    uint32_t magnitude = this->readXU(msb - 1, lsb);
    return this->readXF(msb) ? -magnitude : magnitude;
}
