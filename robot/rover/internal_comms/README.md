## Info
This is the internal communications library. 
It is used by the teensies and the OBC to communicate over UART using a standardized messaging system.

## How it works
The library has two main components. 

The Serial class is responsible for all serial interactions. Reading/Writing and Starting/Stopping the connection.

The CommandCenter class is responsible for (1) taking the serial input and converting it into a usable format, which is a struct that contains relevant information. The CommandCenter does not, at any point, know what is actually contained in the messages. 
It is responsible for (2) generating the replies by combining the data into one struct that is enqueued.
And finally, (2) it is responsible for enqueuing the messages so that they can be processed by Serial.
The class also contains a virtual function that must be implemented. That function is the actual command logic of the teensy.

## How to use it
Read the header and source files to understand what is happening. It is short and somewhat well documented. 
The process is as follows:
1. In the teensy loop, call the `readCommand` method. This will trickle down into your implementation of `executeCommand` if the command is valid.
2. Call `sendMessage` in your loop. It will check the message queue and send a message if the transceiver pin is enabled. 
3. Before sending anything, you must create a message struct using `createMessage` and you must enqueue it using `queueMessage`.
