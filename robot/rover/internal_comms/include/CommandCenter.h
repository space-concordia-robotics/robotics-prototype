#ifndef INTERNAL_COMMS_COMMANDCENTER_H
#define INTERNAL_COMMS_COMMANDCENTER_H
//
// Created by cedric on 2020-10-14.
// And mostly written by Tim
//

#include "Arduino.h"
#include <etl/queue.h>

// USB debug flag
 #define DEBUG

#ifndef DEBUG
#define Serial Serial1
#endif


#define COMMS_BAUDRATE 57600L

//#ifdef UART_PORT
//   #define Serial Serial1
//#elif USB_PORT
//   #define Serial Serial
//#endif

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
            virtual void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) = 0;

            /**
            * Reads serial to next command
            * and returns a command struct
            *
            * @return returns a command struct
            */
            Command* processCommand() ;

            /**
            * Queues messages so that they are ready to be sent
            * allows teensy.
            * Currently we do not do anything if the queue fills up.
            * Hopefully we can find a decent queue size that will never fill.
            * @message reference to message struct
            */
            void queueMessage(Message& message);

            /**
             * @param messageID command ID of message to send
             * @param rawArgsLength number of arguments in the message
             * @param rawArgs contains raw bytes of arguments
             * @return returns a message struct
             */
            Message* createMessage(int messageID, int rawArgsLength, byte *rawArgs);


            /**
            * Holds the messages that are ready to be sent out 
            */
            etl::queue<Message, 5> messageQueue;

            /**
             * Starts serial connection with given pins. Enable pin is for 485 flow control
             */
            void startSerial(uint8_t rxPin, uint8_t txPin, uint8_t enablePin, uint8_t transmitPin);

            /**
             * Reads data from serial port
             */
            void readCommand();

            /**
             * Sends first item of the queue if anything
             */
            void sendMessage();

            /**
             * Enqueues the message and sends top message if enablePin is high
             */
            void sendMessage(Message& message);

            /**
             * Closes serial connection
             */
            void endSerial();


            /**
             * Send debug message string
             */
            void sendDebug(const char* debugMessage);

        private:
            uint8_t enablePin;


            /**
            * Reads the two bytes that make up the argument length and combines
            * them into an uint16_t
            */
            uint16_t readArgSize();

            /**
             * Waits until there is something to read or for 50ms,
             * whichever comes first.
             */
            uint8_t waitForSerial();

    };

}
#endif //INTERNAL_COMMS_COMMANDCENTER_H
