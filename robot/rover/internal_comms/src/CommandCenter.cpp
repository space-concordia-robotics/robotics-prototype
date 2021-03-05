//
// Created by cedric on 2020-10-14.
//

#include "include/CommandCenter.h"
#include <etl/queue.h>
#include "include/Serial.h"

namespace internal_comms
{
    Command* CommandCenter::processCommand() const
    {
        uint8_t commandID = Serial.read();
        uint8_t deviceSending = Serial.read();
        uint8_t deviceReceiving = Serial.read();
        uint16_t argumentSize = readArgSize();

        uint8_t* buffer = (uint8_t*) malloc(sizeof(uint8_t) * argumentSize);
        uint16_t bytesRead = (uint8_t) Serial.readBytes((char*)buffer, argumentSize);
        uint8_t stopByte = Serial.read();

        Command* cmd = (Command*) malloc(sizeof(Command));
        cmd->commandID = commandID;
        cmd->isValid = true;
        cmd->rawArgs = buffer;
        cmd->rawArgsLength = bytesRead;

        if(bytesRead != argumentSize)
        {
            // Possibly an issue
        }

        if(stopByte != 0)
        {
            //Possibly an issue
        }
        
        return cmd;
    }

    uint16_t CommandCenter::readArgSize() const 
    {
        // Serial.read() returns size_t
        uint16_t byte1 = Serial.read();
        uint8_t byte2 = Serial.read();
        uint16_t ArgumentsLength = (byte1 << 8) | byte2;
        return ArgumentsLength;
    }
    
    void CommandCenter::queueMessage(Message& message)
    {
        messageQueue.push(message);
    }

    bool CommandCenter::checkQueue() const {
        return !messageQueue.empty() ? true : false;
    }
}
