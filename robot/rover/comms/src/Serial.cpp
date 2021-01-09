#include "Serial.h"

namespace internal_comms {

    void startSerial(uint8_t rxPin, uint8_t txPin, long baudRate)
    {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT); 

        Serial.begin(baudRate);
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
