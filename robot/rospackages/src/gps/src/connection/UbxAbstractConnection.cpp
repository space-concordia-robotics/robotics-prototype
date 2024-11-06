#include <cstdint>
#include <array>
#include <vector>

#include <thread>
#include <chrono>

#include "../packet/UbxPacket.hpp"
#include "UbxAbstractConnection.hpp"

void UbxAbstractConnection::delayms(int time)
{
    if(time <= 0)
    {
        return; 
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

bool UbxAbstractConnection::sendPacket(UbxPacket& packet)
{
    std::vector<uint8_t> frame = packet.serialize(); 
    for(int i = 0; i < this->nTxRetry; i++)
    {
        bool status = this->sendFrame(frame); 
        if(status == true)
        {
            return true; 
        }
        this->delayms(this->nTxRetryDly); 
    }
    return false; 
}

template <typename T>
typename std::enable_if<std::is_base_of<UbxPacket, T>::value, bool>::type
UbxAbstractConnection::recvPacket(T& packet)
{
    std::vector<uint8_t> frame; 
    for(int i = 0; i < this->nRxRetry; i++)
    {
        bool status = this->recvFrame(frame); 
        if(status == true)
        {
            break; 
        }
        this->delayms(this->nRxRetryDly); 
    }
    if(frame.size() == 0)
    {
        return false; 
    }
    T newPacket {frame}; 
    packet = newPacket; 
    return true; 
}

