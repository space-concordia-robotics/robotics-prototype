## communication

### Tests
All the tests mentioned here are located in tests/ folder

#### 1: Arduino/PC communications
- Upload `ArduinoPC.ino` sketch to your arduino
- Make sure you have the `pyserial` (_not_ `serial`) module installed, use `pip install pyserial`
- Run `sudo python ComArduino.py`

This Ardunio/PC communication demo was found [here](https://forum.arduino.cc/index.php?topic=225329.0)

#### 2: ArdunioListener
- Upload any Arduino code that produces some serial output (such as SerialOutputTest.ino)
- Make sure that the baudrate in ArduinoListener.py is set to the same as the Arduino code you just uploaded
- Run `sudo python ArduinoListener.py` and you should see that output printed onto the terminal, as well as logged into a .dat file

### 3: ArduinoSpeaker
- This test program is meant to communicate with a specific missing arduino code that we are trying to reverse engineer based off of
last year's team's lidar endeavors. The arduino sketch that is meant to be uploaded to work with this program is called `Continuous-Scan.ino`
and if you can't find it in the lidar folder then checkout the `init-lidar` branch. 
