//
// Created by cedric on 2020-10-14.
//

#include "include/CommandCenter.h"

namespace internal_comms
{
    Command* CommandCenter::processCommand() const
    {
        uint8_t commandID = Serial.read();
        uint16_t messageLength = readMessageSize();
        uint8_t* buffer = (uint8_t*) malloc(sizeof(uint8_t) * messageLength);
        uint8_t bytesRead = (uint8_t) Serial.readBytes((char*)buffer, messageLength);

        if(bytesRead != messageLength)
        {
            // Possibly an issue
        }


        Command* cmd = (Command*) malloc(sizeof(Command));
        cmd->commandID = commandID;
        cmd->isValid = true;
        cmd->rawArgs = buffer;
        cmd->rawArgsLength = bytesRead;
        return cmd;
    }

    uint16_t CommandCenter::readMessageSize() const 
    {
        // Serial.read() returns size_t
        uint16_t byte1 = Serial.read();
        uint8_t byte2 = Serial.read();
        uint16_t messageLength = (byte1 << 8) | byte2;
        return messageLength;
    }
}
