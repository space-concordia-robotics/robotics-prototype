#include "UbxI2CConnection.hpp"

#include <cstdint>

#include "LinuxI2CTransactionBuilder.hpp"

#include "UbxAbstractConnection.hpp"
#include "packet/UbxPacket.hpp"


UbxI2CConnection::UbxI2CConnection(const std::string& devicePath, uint8_t devaddr) :
    devaddr{devaddr},
    fd{LinuxI2CTransactionBuilder::openDevice(devicePath)}
{
}

UbxI2CConnection::~UbxI2CConnection()
{
    LinuxI2CTransactionBuilder::closeDevice(this->fd);
}

int UbxI2CConnection::bytesAvailable() const
{
    LinuxI2CTransactionBuilder t{this->devaddr};
    std::array<uint8_t, 2> readBuffer{};

    t.writeArray({0xFD});
    t.readArray(readBuffer);
    t.doTransaction(this->fd);

    return (int)readBuffer[0] << 8 | (int)readBuffer[1];
}

bool UbxI2CConnection::recvFrame(std::vector<uint8_t>& frame)
{
    int bytesToRead = this->bytesAvailable();
    if (bytesToRead <= 0)
    {
        // No bytes available
        return false;
    }
    // Read bytes
    std::vector<uint8_t> buf(bytesToRead);
    for (int bytesToReadNow = bytesToRead <= MAX_TRANSACTION_SIZE ? bytesToRead : MAX_TRANSACTION_SIZE;
         bytesToRead > 0;
         bytesToRead -= bytesToReadNow)
    {
    }
    return true;
}

bool UbxI2CConnection::sendFrame(std::vector<uint8_t>& frame)
{
    // TODO: josh
    return false;
}
