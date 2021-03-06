//
// Created by cedric on 2021-01-16
//

#include "Serial.h"

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
    }

    void sendCommand(Command* command)
    {
        Serial.write(command->commandID);
        Serial.write(command->rawArgsLength);
        Serial.write(command->rawArgs, command->rawArgsLength);
    }

    void endSerial()
    {
        Serial.end();
    }
}
