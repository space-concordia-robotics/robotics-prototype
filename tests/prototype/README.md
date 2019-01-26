# Prototype tests

Tests made while prototyping.

## TeensyBlinkLEDSerial

For testing changing mux select lines and sending serial data from odroid tx to the selected serial device.
It is setup so that you only have to use one teensy for testing with the mux, using 4 rx/tx connections.
It was tested using the ros packages `mux_selector` and `serial_cmd`.
