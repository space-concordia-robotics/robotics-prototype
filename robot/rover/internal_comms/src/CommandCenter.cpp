//
// Created by cedric on 2020-10-14.
// And half of it was done by Tim
//

#include "CommandCenter.h"

#include <etl/queue.h>

#include "Arduino.h"

#define COMMAND_DEBUG_MSG 0

namespace internal_comms {

    Command* CommandCenter::processCommand() {
        uint8_t commandID = waitForSerial();
        uint8_t argumentSize = readArgSize();

        uint8_t* buffer = nullptr;
        uint16_t bytesRead = 0;

        if (argumentSize > 0) {
            buffer = (uint8_t*)malloc(sizeof(uint8_t) * argumentSize);
            bytesRead = (uint8_t)Serial.readBytes((char*)buffer, argumentSize);
        }

        uint8_t stopByte = waitForSerial();

        Command* cmd = (Command*)malloc(sizeof(Command));
        cmd->commandID = commandID;
        cmd->isValid = true;
        cmd->rawArgs = buffer;
        cmd->rawArgsLength = bytesRead;

        /*Serial.print("Command id: ");
          Serial.print(commandID, HEX);
          Serial.print(" Argsize: ");
          Serial.print(argumentSize, DEC);
          Serial.print(" Bytes read after argsize ");
          Serial.print(bytesRead, DEC);
          Serial.print(" stopByte: ");
          Serial.print(stopByte, HEX);
          Serial.print("\n");*/

        if (bytesRead != argumentSize) {
            // bad, but no way of dealing with it for now
        }

        if (stopByte != 0x0a) {
            // bad, but no way of dealing with it for now
        }

        return cmd;
    }

    uint16_t CommandCenter::readArgSize() { return waitForSerial(); }

    Message* CommandCenter::createMessage(int messageID, int rawArgsLength,
            byte* rawArgs) {
        Message* message = (Message*)malloc(sizeof(Message));
        message->messageID = messageID;
        message->rawArgsLength = rawArgsLength;
        message->rawArgs = malloc(rawArgsLength);
        memcpy(message->rawArgs, rawArgs, rawArgsLength);
        return message;
    }

    void CommandCenter::queueMessage(Message& message) {
        // We assume that the queue will never fill up
        // Thus, never fail...
        messageQueue.push(message);
    }

    uint8_t CommandCenter::waitForSerial() {
        unsigned long start = millis();
        while (!Serial.available()) {
            unsigned long current = millis() - start;
            if (current > 50) break;
        }
        return Serial.read();
    }

    // No where else uses rxPin and txPin, BUT can't get rid of them without
    // breaking other code that is out of scope for me (Marc)
    void CommandCenter::startSerial(uint8_t rxPin, uint8_t txPin, uint8_t enablePin,
            uint8_t transmitPin) {
        pinMode(enablePin, INPUT);
        CommandCenter::enablePin = enablePin;

        Serial.begin(COMMS_BAUDRATE);

#ifndef DEBUG
        pinMode(transmitPin, OUTPUT);
        Serial.transmitterEnable(
                transmitPin);  // must disable this for testing with USB
#endif
    }

    void CommandCenter::readCommand() {
        if (Serial.available() > 0) {
            Command* command = CommandCenter::processCommand();
            if (command->isValid) {
                if (command->commandID == 255) {
                    // a command ID of 255 means the OBC is asking the teensy to ID itself
                    sendDebug(getIdentifier());
                    sendMessage();
                    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                } else {
                    this->executeCommand(command->commandID, command->rawArgs,
                        command->rawArgsLength);
                }
            } else {
                // Handle invalid command
                // (Create error message and put it at the front of the queue so that the
                // TX2 can resend)
            }

            free(command->rawArgs);
            command->rawArgs = nullptr;

            delete command;
            command = nullptr;
        } else {
            return;
        }
    }

    void CommandCenter::sendDebug(const char* debugMessage) {
        Message* message = this->createMessage(COMMAND_DEBUG_MSG,
                strlen(debugMessage), debugMessage);
        this->queueMessage(*message);
    }

    void CommandCenter::sendMessage() {
        if (digitalRead(enablePin)) {
            if (!messageQueue.empty()) {
                Message message = messageQueue.front();
                messageQueue.pop();

                Serial.write(message.messageID);
                Serial.write(message.rawArgsLength);

                if (message.rawArgsLength > 0)
                    Serial.write(message.rawArgs, message.rawArgsLength);

                Serial.write(0x0A);

                free((void*)message.rawArgs);
            }
        }
    }

    void CommandCenter::sendMessage(Message& message) {
        messageQueue.push(message);
        sendMessage();
    }

    void CommandCenter::endSerial() { Serial.end(); }
}  // namespace internal_comms
