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
2. Witin `executeCommand`, you must create a message struct using `createMessage`.
3. If the commandID from the `executeCommand` is valid, add the created message to a queue using `sendMessage(Message& message)` within the case statement.  
### Note ###
The queue's purpose is to keep track of previous messages, so that they can be performed once the serial pin is enabled for the particular module. This queue stores all messages which were called to be sent, in case the message is received, but unable to be executed due to the UART pin being disabled. A UART pin can be disabled for a particular module if it is being used by a different module at a current moment. With the message added, then the latest message added to the queue is attempted to be sent. This is because the queue of messages must start by sending the latest message, and not the most recent! If this message is the only message in the queue, it is added, sent and removed from the queue.

4. After the message is added to the queue, call the associated .ino function depending on the commandID
5. Within the function for the specific command, call `sendMessage()` in your loop.  
### Note ###
`sendMessage()` will check if the transmission pin is enabled and pop the message out of the queue in order to be sent.  

