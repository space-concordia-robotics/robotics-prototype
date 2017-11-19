## communication

### Current Tasks (phase 3)

Sprint 1:
- programmatically get serial output from arduino over usb connection to PC
- test previous teams networking libraries, look into alternatives, determine based off testing which to use
- help motor/LIDAR to integrate their outputs with GUI

Sprint 2:
shared task with ELEC team:
- create demo to: read values from LIDAR, convert to appropriate format, send this data to the odroid.
Then have odroid (using XBEEs) transmit this data over RF to the basestation. Display the measurement packet info on the basestation.

### Tests
#### Ardunio/PC communication demo found [here](https://forum.arduino.cc/index.php?topic=225329.0)
1. Upload `ArduinoPC.ino` sketch to your arduino
2. Run `ComArduino.py`
- make sure you have the `pyserial` (_not_ `serial`) module installed, use `pip install pyserial`

*NOTE:* The Arduino.cpp and MakeFile files are there for compiling/uploading sketches to aruinos using the command line.
The instructions are in the MakeFile. This can be useful if you can't use the Arduino IDE to compile/upload sketches.

