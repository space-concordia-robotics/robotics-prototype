#pragma onc{e

#include <cstdint>

#include "UbxAbstractConnection.hpp"
#include "../src/packet/UbxPacket.hpp"

class UbxI2CConnection : public UbxAbstractConnection
{
protected:
    int fd;
    uint8_t devaddr;
    int bytesAvailable() const;
    bool recvFrame(std::vector<uint8_t>& frame) override;
    bool sendFrame(std::vector<uint8_t>& frame) override;

public:
    const int MAX_TRANSACTION_SIZE = 32;

    UbxI2CConnection(const std::string& devicePath, uint8_t devaddr);
    ~UbxI2CConnection() override;
};
