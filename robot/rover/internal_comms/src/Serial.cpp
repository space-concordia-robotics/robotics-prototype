//
// Created by cedric on 2021-01-16
//

#include "include/Serial.h"

namespace internal_comms
{
    void startSerial(uint8_t rxPin, uint8_t txPin)
    {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT); 

        Serial.begin(COMMS_BAUDRATE);
    }

    void readCommand(CommandCenter* commandCenter)
    {
        Command* command = commandCenter->processCommand();
        if(command->isValid)
        {
            commandCenter->executeCommand(command->commandID, command->rawArgs, command->rawArgsLength);
        }
        else
        {
            // Handle invalid command
        }

        free(command->rawArgs);
        command->rawArgs = nullptr;

        delete command;
        command = nullptr;
    }

    void sendMessage(Message* message)
    {
        Serial.write(message->messageID);
        Serial.write(message->rawArgsLength);
        Serial.write(message->rawArgs, message->rawArgsLength);
    }

    void endSerial()
    {
        Serial.end();
    }
}
