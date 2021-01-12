//
// Created by cedric on 2020-10-14.
//

#include "include/CommandCenter.h"

namespace internal_comms
{
    Command* CommandCenter::processCommand() const
    {
        uint8_t commandID = Serial.read();
        size_t bytesToRead = Serial.read();
        uint8_t* buffer = (uint8_t*) malloc(sizeof(uint8_t) * bytesToRead);
        uint8_t bytesRead = (uint8_t) Serial.readBytes((char*)buffer, bytesRead);

        if(bytesRead != bytesToRead)
        {
            // Possibly an issue
        }


        Command* com = (Command*) malloc(sizeof(Command));
        com->commandID = commandID;
        com->isValid = true;
        com->rawArgs = buffer;
        com->rawArgsLength = bytesRead;
        return com;
    }
}
