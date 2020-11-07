//
// Created by cedric on 2020-10-14.
//

#include "include/commands/CommandCenter.h"

bool CommandCenter::processCommand(const String rawCommand, String &commandName, String ** arguments) const {
    int commandDelimIndex = rawCommand.indexOf('=');

    if(commandDelimIndex < 1)
        return false;

    commandName = rawCommand.substring(0, commandDelimIndex);
    String args = rawCommand.substring(commandDelimIndex + 1);

    int numberOfArgs = 0;
    String rawCommandCopy = rawCommand;
    while(int argDelimIndex = rawCommand.indexOf(',') < 1)
    {
        rawCommandCopy = rawCommandCopy.substring(argDelimIndex + 1);
        numberOfArgs++;
    }
    numberOfArgs += 1;
    String* lastArg = new String(rawCommandCopy);
    arguments = static_cast<String **>(malloc(sizeof(String *) * numberOfArgs)); // Should not be too bad on memory

    String newCommandCopy = rawCommand;
    int currentArg = 0;
    while(int argDelimIndex = newCommandCopy.indexOf(',') < 1)
    {
        String* arg = new String(newCommandCopy.substring(0, argDelimIndex));
        newCommandCopy = newCommandCopy.substring(argDelimIndex + 1);
        arguments[currentArg] = arg;
        currentArg++;
    }
    arguments[numberOfArgs - 1] = lastArg;

    return true;
}