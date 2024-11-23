#pragma once

#include <cstdint>
#include <string>
#include <vector>

class UbxDeserializerException : public std::exception
{
private:
    std::vector<uint8_t> payload; // these are all read only, can we make optimizations?
    uint32_t readIndex;
    uint32_t readLength;
    std::string message;

public:
    explicit UbxDeserializerException(const std::vector<uint8_t>& payload, uint32_t readIndex, uint32_t readLength,
                                      const std::string& message)
    {
        // Initialize this in a better way?
        this->payload = payload;
        this->readIndex = readIndex;
        this->readLength = readLength;
        this->message = message;
    }

    // TODO
    virtual const char* what() const noexcept override
    {
        // How can I build a message like this:
        // "Error reading index <readIndex> length <readLength>: <message> (<payload bytes in hex>)"
        // Example: "Error reading 4 length 1: Payload drained. (64 00 1A 93)"
    }
};

class UbxPayloadDrainedException : public UbxDeserializerException
{
public:
    // Only change is the constructor now contains the error message
    explicit UbxPayloadDrainedException(const std::vector<uint8_t>& payload, uint32_t readIndex, uint32_t readLength)
        : UbxDeserializerException::UbxDeserializerException(payload, readIndex, readLength, "Payload drained.")
    {
    }
};

class UbxPayloadExcessException : public UbxDeserializerException
{
public:
    // Only change is the constructor now contains the error message
    explicit UbxPayloadExcessException(const std::vector<uint8_t>& payload, uint32_t readIndex, uint32_t readLength)
        : UbxDeserializerException(payload, readIndex, readLength, "Payload has more bytes.")
    {
    }
};


class UbxDeserializer
{
private:
    std::vector<uint8_t> payload;
    uint32_t index;
    uint32_t bitfieldBuffer;

    uint8_t readByteNochk();

public:
    explicit UbxDeserializer(std::vector<uint8_t> payload);
    void reset();
    void assertBytesAvailable(uint32_t length);
    /**
     * @param bytesToSkip number of bytes that will be skipped in payload
     *
     * This function throws a UbxPayloadDrainedException if bytesToSkip is greater than bytes available
     */
    void skipBytes(int bytesToSkip) noexcept(false);
    /**
     * This function throws a UbxPayloadDrainedException if bytesToSkip is greater than bytes available
     */
    void skip1Byte();
    void skip2Bytes();
    void skip4Bytes();
    /**
    *
    * This function throws a UbxPayloadExcessException if index > payload size
    */
    void assertEmpty();
    uint8_t readU1();
    int8_t readI1();
    uint16_t readU2();
    int16_t readI2();
    uint32_t readU4();
    int32_t readI4();
    void readByteArray(uint8_t* data, uint32_t length);
    std::vector<uint8_t> readVector(uint32_t length);
    float readR4();
    double readR8();
    char readCh();
    void readX1();
    void readX2();
    void readX4();
    bool readXF(int bit);
    uint32_t readXU(int msb, int lsb);
    int32_t readXI(int msb, int lsb);
    int32_t readXS(int msb, int lsb);
};
