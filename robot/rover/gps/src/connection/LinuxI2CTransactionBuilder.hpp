#pragma once

#include <cstdint>
#include <array>
#include <vector>

#include <linux/i2c.h>


class LinuxI2CTransactionBuilder {
 private:
	std::vector<i2c_msg> segments;

	i2c_msg buildPartialSegment(uint8_t* buffer, int length);

 public:
	static int openDevice(const std::string& path);

	static void closeDevice(int fd);

	const uint8_t devAddr;

	explicit LinuxI2CTransactionBuilder(uint8_t devAddr);

	LinuxI2CTransactionBuilder(uint8_t devAddr, int reserveSize);

	LinuxI2CTransactionBuilder& writeRawPtr(uint8_t* buffer, int length);

	LinuxI2CTransactionBuilder& writeVector(std::vector<uint8_t>& buffer);

	template<std::size_t N>
	LinuxI2CTransactionBuilder& writeArray(std::array<uint8_t, N>& buffer);

	// TODO
	LinuxI2CTransactionBuilder& writeArray(std::initializer_list<uint8_t> initializerList);

	template<typename T>
	LinuxI2CTransactionBuilder& writeAny(T&& data);

	LinuxI2CTransactionBuilder& readRawPtr(uint8_t* buffer, int length);

	LinuxI2CTransactionBuilder& readVector(std::vector<uint8_t>& buffer);

	template<std::size_t N>
	LinuxI2CTransactionBuilder& readArray(std::array<uint8_t, N>& buffer);

	template<typename T>
	LinuxI2CTransactionBuilder& readAny(T& data);

	int doTransaction(int fd);
};
