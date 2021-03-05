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
            // (Create error message and put it at the front of the queue so that the 
            // TX2 can resend)
        }

        free(command->rawArgs);
        command->rawArgs = nullptr;

        delete command;
        command = nullptr;
    }

    void sendMessage(CommandCenter* commandCenter)
    {
        if (commandCenter->checkQueue()) {

            const Message& message = commandCenter->messageQueue.front();
            commandCenter->messageQueue.pop();


            Serial.write(message.messageID);
            Serial.write(message.rawArgsLength);
            Serial.write(message.rawArgs, message.rawArgsLength);

            // Then we probably should delete the message or store it somewhere for a few cycles

        } else {
            Serial.write("something"); // idk what to send but it should basically say that there is nothing to send.
        }
    }

    void endSerial()
    {
        Serial.end();
    }
}
