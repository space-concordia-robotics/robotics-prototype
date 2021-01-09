//
// Created by cedric on 2020-10-14.
//

#ifndef INTERNAL_COMMS_COMMANDCENTER_H
#define INTERNAL_COMMS_COMMANDCENTER_H
#include "Arduino.h"

namespace internal_comms
{
    typedef struct {
        uint8_t commandID; // ID of command
        uint8_t* rawArgs; // Byte array with the bytes of the arguments
        bool isValid; // Whether the command is valid
    } Command;

    class CommandCenter {
        public:

            /**
             * Executes the command with the commandID with the raw arguments
             * @param commandID The ID of the command to execute
             * @param rawArgs The raw bytes consisting of the arguments
             */
            virtual void executeCommand(const uint8_t commandID, const uint8_t* rawArgs) = 0;

            /**
             * Takes as an input a rawCommand.
             * Parses the command and returns by reference a command struct
             *
             * @param rawCommand
             * @return returns a command struct
             */
            virtual Command* processCommand(const uint8_t* rawCommand) const;
    };

}
#endif //INTERNAL_COMMS_COMMANDCENTER_H
