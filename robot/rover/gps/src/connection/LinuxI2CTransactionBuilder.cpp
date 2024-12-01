#include <iostream>
#include <cstdint>
#include <cstring>
#include <cerrno>

#include <array>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "LinuxI2CTransactionBuilder.hpp"

int LinuxI2CTransactionBuilder::openDevice(const std::string& path)
{
	// "/dev/i2c-x"
	int fd = open(path.c_str(), O_RDWR);
	if (fd < 0) {
		// TODO: throw exception?
		std::cerr << "Failed to open i2C device " << path << ". Error: " << strerror(errno) << std::endl;
		errno = 0;
		return -1;
	}
	return fd;
}

void LinuxI2CTransactionBuilder::closeDevice(int fd)
{
	int status = close(fd);
	if (fd < 0) {
		// TODO: throw exception?
		std::cerr << "Failed to close i2C device. Error: " << strerror(errno) << std::endl;
		errno = 0;
	}
}

LinuxI2CTransactionBuilder::LinuxI2CTransactionBuilder(uint8_t devAddr) :
	LinuxI2CTransactionBuilder(devAddr, 1)
{
}

LinuxI2CTransactionBuilder::LinuxI2CTransactionBuilder(uint8_t devAddr, int reserveSize) :
	devAddr{devAddr}
{
	this->segments.reserve(reserveSize);
}

// Build internal segment
struct i2c_msg LinuxI2CTransactionBuilder::buildPartialSegment(uint8_t* buffer, int length)
{
	struct i2c_msg segment;
	segment.addr = this->devAddr;
	segment.buf = buffer;
	segment.len = length;
	return segment;
}

// Write with raw pointer
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::writeRawPtr(uint8_t* buffer, int length)
{
	struct i2c_msg segment = this->buildPartialSegment(
		buffer,
		length
	);
	segment.flags = 0;
	this->segments.push_back(segment);
	return *this;
}

// Read into raw pointer
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::readRawPtr(uint8_t* buffer, int length)
{
	struct i2c_msg segment = this->buildPartialSegment(
		buffer,
		length
	);
	segment.flags = I2C_M_RD;
	this->segments.push_back(segment);
	return *this;
}

// Write with vector container
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::writeVector(std::vector<uint8_t>& buffer)
{
	return this->writeRawPtr(buffer.data(), buffer.size());
}

// Read into vector container
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::readVector(std::vector<uint8_t>& buffer)
{
	return this->readRawPtr(buffer.data(), buffer.size());
}

// Write with array container / literals
template<std::size_t N>
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::writeArray(std::array<uint8_t, N>& buffer)
{
	return this->writeRawPtr(buffer.data(), N);
}

LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::writeArray(std::initializer_list<uint8_t> initializerList)
{
	std::vector<uint8_t> buffer(initializerList);
	return this->writeVector(buffer);
}

// Read into array container
template<std::size_t N>
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::readArray(std::array<uint8_t, N>& buffer)
{
	return this->readRawPtr(buffer.data(), N);
}

// Write with arbitrary value / reference
template<typename T>
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::writeAny(T&& data)
{
	return this->writeRawPtr(reinterpret_cast<uint8_t*>(&data), sizeof(data));
}

// Read into arbitrary reference
template<typename T>
LinuxI2CTransactionBuilder& LinuxI2CTransactionBuilder::readAny(T& data)
{
	return this->readRawPtr(reinterpret_cast<uint8_t*>(&data), sizeof(data));
}

int LinuxI2CTransactionBuilder::doTransaction(int fd)
{
	if (this->segments.size() == 0) {
		// Nothing to perform. Return as successful.
		return 0;
	}

	struct i2c_rdwr_ioctl_data transaction;
	transaction.msgs = segments.data();
	transaction.nmsgs = segments.size();

	// Perform compound I2C transaction
	int status = ioctl(fd, I2C_RDWR, &transaction);
	if (status < 0) {
		// TODO: print out more information on the transaction?
		// TODO: make this an exception?
		std::cerr << "[I2C Error] Transaction error " << strerror(errno) << std::endl;
		errno = 0;
		return errno;
	}
	return 0;
}

/* -------- Test code follows -------- */

#include <iomanip>
#include <string>
#include <chrono>
#include <thread>

using namespace std;

//int main()
//{
//	int fd = LinuxI2CTransactionBuilder::openDevice("/dev/i2c-7");
//	if (fd < 0) {
//		cout << "Error: Unable to open I2C device. Terminating." << endl;
//		return 0;
//	}
//
//	// Polling packet for UBX-NAV-POSLLH
//	uint8_t packet[] = {
//		0xB5, 0x62, // Preamble
//		0x01, 0x02, // Compound PID
//		0x00, 0x00, // Length (little endian)
//		// Payload:
//
//	};
//
//	vector<uint8_t> v;
//	uint8_t A = 0, B = 0;
//	for (int i = 0; i < sizeof(packet); i++) {
//		v.push_back(packet[i]);
//		if (i >= 2) {
//			A += packet[i];
//			B += A;
//		}
//	}
//	v.push_back(A);
//	v.push_back(B);
//
//	cout << "Packet: ";
//	for (uint8_t ch0 : v) {
//		uint32_t ch = (uint32_t) ch0;
//		cout << hex << setw(2) << setfill('0') << ch << " ";
//	}
//	cout << endl;
//
//	LinuxI2CTransactionBuilder t(0x42);
//	t.writeVector(v).doTransaction(fd);
//	for (int i = 0; i < 50; i++) {
//		LinuxI2CTransactionBuilder p(0x42);
//		uint8_t reg = 0xFD;
//		uint16_t len = 0;
//		p.writeAny(reg).readAny(len).doTransaction(fd);
//		len = (len >> 8) | (len << 8);
//		if (len == 0) {
//			std::this_thread::sleep_for(std::chrono::milliseconds(10));
//			continue;
//		}
//		LinuxI2CTransactionBuilder q(0x42);
//		vector<uint8_t> s;
//		s.resize(len);
//		q.readVector(s).doTransaction(fd);
//		cout << "Received " << s.size() << " bytes: ";
//		for (auto ch0 : s) {
//			uint32_t ch = (uint32_t) ch0;
//			cout << hex << setw(2) << setfill('0') << ch << " ";
//		}
//		cout << endl;
//	}
//
//	/*
//		cout << "Please enter device address (GPS is 0x42, 0d66): ";
//		int devaddr0;
//		cin >> devaddr0;
//		uint8_t devaddr = static_cast<uint8_t>(devaddr0);
//		cout << endl;
//		if(devaddr < 0 || devaddr > 0x7F)
//		{
//			cout << "Error: Device address invalid." << endl;
//			return 0;
//		}
//
//		cout << "Please enter transaction type. 1: WR, 2: RD, 3: WR+RD: ";
//		int type;
//		cin >> type;
//		cout << endl;
//		if(type < 1 || type > 3)
//		{
//			cout << "Error: Transaction type invalid." << endl;
//			return 0;
//		}
//
//		I2CTransactionBuilder transaction {devaddr};
//
//		vector<uint8_t> wrData;
//		vector<uint8_t> rdData;
//
//		if(type & 0x1) // Handle write
//		{
//			cout << "Enter bytes to write." << endl << "> ";
//			cin.ignore(10000, '\n');
//			string hexstr;
//			getline(std::cin, hexstr);
//			cout << endl;
//			// Parse string
//			bool readingMsb = true; // start of number
//			uint8_t temp = 0;
//			for(char ch : hexstr)
//			{
//				bool valid = false;
//				uint8_t num = 0;
//				if(ch >= '0' && ch <= '9')
//				{
//					valid = true;
//					num = ch - '0';
//				}
//				if(ch >= 'a' && ch <= 'f')
//				{
//					valid = true;
//					num = ch - 'a' + 10;
//				}
//				if(ch >= 'A' && ch <= 'F')
//				{
//					valid = true;
//					num = ch - 'A' + 10;
//				}
//				if(readingMsb)
//				{
//					if(!valid)
//					{
//						continue;
//					}
//					temp = num;
//					readingMsb = false;
//				}
//				else
//				{
//					if(!valid)
//					{
//						num = temp;
//					}
//					else
//					{
//						num = num | temp << 4;
//					}
//					wrData.push_back(num);
//					readingMsb = true;
//				}
//			}
//			if(wrData.size() == 0)
//			{
//				cout << "Error: Nothing to write." << endl;
//				return 0;
//			}
//			// Load segment
//			transaction.writeVector(wrData);
//		}
//		if(type & 0x2) // Handle read
//		{
//			int size;
//			cout << "Enter bytes to read: ";
//			cin >> size;
//			cout << endl;
//			if(size <= 0)
//			{
//				cout << "Error: Nothing to read." << endl;
//				return 0;
//			}
//			rdData.resize(size);
//			// Load segment
//			transaction.readVector(rdData);
//		}
//
//		// Transaction preview
//		cout << "Transaction preview: " << endl;
//		if(type & 0x1)
//		{
//			cout << "S ADDR(W) ";
//			for(uint8_t d : wrData)
//			{
//				int d0 = static_cast<int>(d);
//				cout << hex << setw(2) << setfill('0') << d0 << " ";
//			}
//		}
//		if(type & 0x2)
//		{
//			cout << "S ADDR(R) ";
//			for(uint8_t ignored : rdData)
//			{
//				cout << "XX ";
//			}
//		}
//		cout << "P" << endl;
//
//		cout << "Press any key to continue." << endl;
//		std::cin.ignore(1000000, '\n');
//		string str;
//		cin >> str;
//		// Perform transaction
//		transaction.doTransaction(fd);
//		cout << "Transaction performed." << endl;
//		if(type & 0x2)
//		{
//			cout << "Data read: ";
//			for(uint8_t d : rdData)
//			{
//				int d0 = static_cast<int>(d);
//				cout << hex << setw(2) << setfill('0') << d0 << " ";
//			}
//			cout << endl;
//		}
//
//		I2CTransactionBuilder::closeDevice(fd);
//	*/
//	return 0;
//}

