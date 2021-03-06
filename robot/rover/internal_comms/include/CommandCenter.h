#ifndef INTERNAL_COMMS_COMMANDCENTER_H
#define INTERNAL_COMMS_COMMANDCENTER_H
//
// Created by cedric on 2020-10-14.
//

#include "Arduino.h"

namespace internal_comms
{
    typedef struct {
        uint8_t commandID; // ID of command
        uint8_t* rawArgs; // Byte array with the bytes of the arguments
        uint16_t rawArgsLength; // Number of bytes in the rawArgs array
        bool isValid; // Whether the command is valid
    } Command;

    class CommandCenter {
        public:

            /**
             * Executes the command with the commandID with the raw arguments
             * @param commandID The ID of the command to execute
             * @param rawArgs The raw bytes consisting of the arguments
             */
            virtual void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) = 0;

            /**
             * Reads serial the next command
             * and returns a command struct
             *
             * @return returns a command struct
             */
            Command* processCommand() const;

        private:
            uint16_t readArgSize() const;
    };

    uint8_t waitForSerial();
}
#endif //INTERNAL_COMMS_COMMANDCENTER_H
