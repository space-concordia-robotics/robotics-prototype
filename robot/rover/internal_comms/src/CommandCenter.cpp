//
// Created by cedric on 2020-10-14.
// And half of it was done by Tim
//

#include "CommandCenter.h"
#include <etl/queue.h>

namespace internal_comms
{
    Command* CommandCenter::processCommand()
    {
        uint8_t commandID = waitForSerial();
        uint8_t deviceSending = waitForSerial();
        uint8_t deviceReceiving = waitForSerial();
        uint16_t argumentSize = readArgSize();

        uint8_t* buffer = (uint8_t*) malloc(sizeof(uint8_t) * argumentSize);
        uint16_t bytesRead = (uint8_t) Serial.readBytes((char*)buffer, argumentSize);
        uint8_t stopByte = waitForSerial();

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

    uint16_t CommandCenter::readArgSize() {
        // Serial.read() returns size_t
        uint16_t byte1 = waitForSerial();
        uint8_t byte2 = waitForSerial();
        uint16_t ArgumentsLength = (byte1 << 8) | byte2;
        return ArgumentsLength;
    }

    Message* CommandCenter::createMessage(int messageID, int rawArgsLength, byte* rawArgs) {
        Message* message = (Message*) malloc(sizeof(Message));
        message->messageID = messageID;
        message->rawArgsLength = rawArgsLength;
        message->rawArgs = rawArgs;
        return message;
    }

    void CommandCenter::queueMessage(Message& message) {
        // We assume that the queue will never fill up
        // Thus, never fail...
        messageQueue.push(message);
    }

    uint8_t CommandCenter::waitForSerial() {
        unsigned long start = millis();
        while(!Serial.available()){
            unsigned long current = millis() - start;
            if (current > 50) break;
        }
        return Serial.read();
    }
    
    void CommandCenter::startSerial(uint8_t rxPin, uint8_t txPin, uint8_t enablePin)
    {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT); 
        pinMode(enablePin, INPUT);
        CommandCenter::enablePin = enablePin;

        Serial.begin(COMMS_BAUDRATE);
    }

    void CommandCenter::readCommand()
    {
        Command* command = CommandCenter::processCommand();
        if(command->isValid)
        {
            this->executeCommand(command->commandID, command->rawArgs, command->rawArgsLength);
        }
        else
        {
            // Handle invalid command
            // (Create error message and put it at the front of the queue so that the 
            // TX2 can resend)
        }

        free(command->rawArgs);
        command->rawArgs = nullptr;

        delete command;
        command = nullptr;
    }

    void CommandCenter::sendMessage() {
        if (digitalRead(enablePin)) {
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
    }

    void CommandCenter::sendMessage(Message& message) {
        messageQueue.push(message);
        sendMessage();
    }

    void CommandCenter::endSerial()
    {
        Serial.end();
    }
}

