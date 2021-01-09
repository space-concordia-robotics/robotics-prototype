//
// Created by cedric on 2020-10-14.
//

#ifndef INTERNAL_COMMS_COMMANDCENTER_H
#define INTERNAL_COMMS_COMMANDCENTER_H
#include "Arduino.h"

namespace internal_comms
{
    typedef struct {
        byte commandID,
             byte* rawArgs,
             bool isValid
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
             * Takes as an input a rawCommand in the form commandName=arg1,arg2,arg3,...
             * Parses the command and returns by reference commandName and a const char* array of arguments
             *
             * @param rawCommand
             * @return returns true if the command is valid, false otherwise
             */
            virtual Command* processCommand(const byte* rawCommand, byte& commandID, byte* rawArgs) const;
    };

}
#endif //INTERNAL_COMMS_COMMANDCENTER_H
