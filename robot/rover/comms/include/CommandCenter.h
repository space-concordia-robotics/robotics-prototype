//
// Created by cedric on 2020-10-14.
//

#ifndef INTERNAL_COMMS_COMMANDCENTER_H
#define INTERNAL_COMMS_COMMANDCENTER_H
#include "Arduino.h"

namespace internal_comms
{
    typedef struct {
        byte commandID, // ID of command
        byte* rawArgs, // Byte array with the bytes of the arguments
        bool isValid // Whether the command is valid
    } Command;

    class CommandCenter {
        public:

            /**
             * Executes the command with the commandID with the raw arguments
             * @param commandID The ID of the command to execute
             * @param rawArgs The raw bytes consisting of the arguments
             */
            virtual void executeCommand(const byte commandID, const byte* rawArgs) = 0;

            /**
             * Takes as an input a rawCommand.
             * Parses the command and returns by reference a command struct
             *
             * @param rawCommand
             * @return returns a command struct
             */
            virtual Command* processCommand(const byte* rawCommand) const;
    };

}
#endif //INTERNAL_COMMS_COMMANDCENTER_H
