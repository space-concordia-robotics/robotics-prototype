//
// Created by cedric on 2020-10-14.
// And half of it was done by Tim
//

#include "CommandCenter.h"
#include <etl/queue.h>

namespace internal_comms
{
    Command* CommandCenter::processCommand() const
    {
        uint8_t commandID = waitForSerial();
        uint8_t deviceSending = waitForSerial();
        uint8_t deviceReceiving = waitForSerial();
        uint16_t argumentSize = readArgSize();

        auto* buffer = (uint8_t*) malloc(sizeof(uint8_t) * argumentSize);
        uint16_t bytesRead = (uint8_t) Serial.readBytes((char*)buffer, argumentSize);
        uint8_t stopByte = waitForSerial();

        auto* cmd = (Command*) malloc(sizeof(Command));
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
        uint16_t byte1 = waitForSerial();
        uint8_t byte2 = waitForSerial();
        uint16_t ArgumentsLength = (byte1 << 8) | byte2;
        return ArgumentsLength;
    }
    
    void CommandCenter::queueMessage(Message& message)
    {
        // We assume that the queue will never fill up
        // Thus, never fail...
        messageQueue.push(message);
    }

    void CommandCenter::sendMessage() {
        if (!messageQueue.empty()) {
            Message message = messageQueue.front();
            messageQueue.pop();

            Serial.write(message.messageID);
            Serial.write(message.rawArgsLength);
            Serial.write(message.rawArgs, message.rawArgsLength);

            delete &message;

        } else {
            Serial.write(1);
            Serial.write(0);
        }
    }

    // Wait until there is something to read or 50ms have gone by
    uint8_t waitForSerial() {
        unsigned long start = millis();
        while(!Serial.available()){
            unsigned long current = millis() - start;
            if (current > 50) break;
        }
        return Serial.read();
    }
}
