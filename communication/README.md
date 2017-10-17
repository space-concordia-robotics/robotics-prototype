## communication

### Current Tasks

- make efforts to locate missing communications components (one missing XBEE tranceiver)
- run a basic test outlining basic elements of communications
- test previous teams networking libraries, look into alternatives
- determine which networking library, hardware to use
- help motor/LIDAR to integrate their outputs with GUI

### Tests
#### Ardunio/PC communication demo found [here](https://forum.arduino.cc/index.php?topic=225329.0)
1. Upload `ArduinoPC.ino` sketch to your arduino
2. Run `ComArduino.py`
- make sure you have the `pyserial` (_not_ `serial`) module installed, use `pip install pyserial`

*NOTE:* The Arduino.cpp and MakeFile files are there for compiling/uploading sketches to aruinos using the command line.
The instructions are in the MakeFile. This can be useful if you can't use the Arduino IDE to compile/upload sketches.

