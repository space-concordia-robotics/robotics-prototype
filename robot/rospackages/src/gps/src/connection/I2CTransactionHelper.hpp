#pragma once

#include <cstdint>
#include <array>
#include <vector>

#include <linux/i2c.h>

class I2CTransactionHelper
{
private:
    std::vector<struct i2c_msg> segments;
    struct i2c_msg buildPartialSegment(uint8_t* buffer, int length);

public:
    static int openDevice(const std::string& path);
    static void closeDevice(int fd);

    const uint8_t devAddr;

    explicit I2CTransactionHelper(uint8_t devAddr);
    I2CTransactionHelper(uint8_t devAddr, int reserveSize);

    I2CTransactionHelper& writeRawPtr(uint8_t* buffer, int length);
    I2CTransactionHelper& writeVector(std::vector<uint8_t>& buffer);

    template <std::size_t N>
    I2CTransactionHelper& writeArray(std::array<uint8_t, N>& buffer);
    I2CTransactionHelper& I2CTransactionHelper::writeArray(std::initializer_list<uint8_t> ilist);

    template <typename T>
    I2CTransactionHelper& writeAny(T&& data);

    I2CTransactionHelper& readRawPtr(uint8_t* buffer, int length);
    I2CTransactionHelper& readVector(std::vector<uint8_t>& buffer);
    template <std::size_t N>
    I2CTransactionHelper& readArray(std::array<uint8_t, N>& buffer);
    template <typename T>
    I2CTransactionHelper& readAny(T& data);

    int doTransaction(int fd);
};

