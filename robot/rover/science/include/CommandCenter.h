//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_COMMANDCENTER_H
#define ROVER_COMMANDCENTER_H


class CommandCenter {
    public:

        /**
         * Executes the command with the name command with the arguments args
         * @param commandName The name of the command to execute
         * @param args The array of arguments as const char*
         */
        virtual void executeCommand(const char* commandName, const char** args);

        /**
         * Takes as an input a rawCommand in the form commandName=arg1,arg2,arg3,...
         * Parses the command and returns by reference commandName and a const char* array of arguments
         *
         * @param rawCommand
         * @return returns true if the command is valid or not
         */
        virtual bool processCommand(const char* rawCommand, char* commandName, char** arguments) const;
};


#endif //ROVER_COMMANDCENTER_H
