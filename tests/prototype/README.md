# Prototype tests

Tests made while prototyping.

## TeensyBlinkLEDSerial

This tests changing mux select lines and sending serial data from odroid tx to the selected serial device.
It is setup so that you only have to use one teensy for testing with the mux, using 4 rx/tx connections.
It was tested using the ros packages `mux_selector` and `serial_cmd`.

The serial messages that are expected on the teensy are simply `1` or `0` and 4 UART serial rx channels are linked to sending a signal to correspondingly turn OFF/ON external 4 LEDs, respectively.

## TeensyToTeensySerial

This tests a speaker/listener using UART communication. 

The speaker sends a message every second and the listener responds to it by blinking an LED a certain amount of times, based off the message sent.
