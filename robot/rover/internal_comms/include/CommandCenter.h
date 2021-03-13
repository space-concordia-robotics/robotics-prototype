#ifndef INTERNAL_COMMS_COMMANDCENTER_H
#define INTERNAL_COMMS_COMMANDCENTER_H
//
// Created by cedric on 2020-10-14.
// And mostly written by Tim
//

#include "Arduino.h"
#include <etl/queue.h>

namespace internal_comms
{
   typedef struct {
       uint8_t commandID; // ID of command
       uint8_t* rawArgs; // Byte array with the bytes of the arguments
       uint16_t rawArgsLength; // Number of bytes in the rawArgs array
       bool isValid; // Whether the command is valid
   } Command;

    typedef struct {
        uint8_t messageID;
        uint16_t rawArgsLength;
        uint8_t* rawArgs;
    } Message;

    class CommandCenter {
        public:

            /**
            * Executes the command with the commandID with the raw arguments
            * @param commandID The ID of the command to execute
            * @param rawArgs The raw bytes consisting of the arguments
            */
            virtual void executeCommand(const uint8_t commandID,
                                        const uint8_t* rawArgs, 
                                        const uint8_t rawArgsLength) = 0;

            /**
            * Reads serial to next command
            * and returns a command struct
            *
            * @return returns a command struct
            */
            Command* processCommand() const;

            /**
            * Queues messages so that they are ready to be sent
            * allows teensy.
            * Currently we do not do anything if the queue fills up.
            * Hopefully we can find a decent queue size that will never fill.
            */
            void queueMessage(Message& message);

            /**
            * Calls Serial::send() if there is something in the queue
            * and frees up the memory
            */
            void sendMessage();


            /**
            * Holds the messages that are ready to be sent out 
            */
            etl::queue<Message, 5> messageQueue;

        private:

            /**
            * Reads the two bytes that make up the argument length and combines
            * them into an uint16_t
            */
            uint16_t readArgSize() const;
    };

    uint8_t waitForSerial();
}

#endif //INTERNAL_COMMS_COMMANDCENTER_H
