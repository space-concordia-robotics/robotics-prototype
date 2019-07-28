# Rover
The code running on the rover

## config
This folder contains some configuration stuff for the Odroid and a sample .bash_aliases file for it as well. See the README inside the folder for more info.

## Arm
### Classes

The Teensy code for the Arm is abstracted into classes to aid in code modularity. Currently there is the RobotMotor class with the DcMotor, StepperMotor (not used anymore) and ServoMotor subclasses. There are also the PidController and Parser classes, and pinSetup which is not a class but is separated into its own .cpp and .h files. In the future, these classes could be turned into Arduino libraries. At the very least, a similar structure can be used on the rover Teensy.

### Header files

Includes.h holds many of the macros and include directives so that the code in Arm.ino is less bloated
Vsense.h holds battery voltage sensing functionality
Notes.h and Ideas.h hold sample code and ideas for future improvements.

### Hardware and Wiring

The current iteration of the rover arm has m1 to m4 being DC motors and m5 and m6 being continuous rotation servos. Currently the DC motors are driven with Cytron drivers and the servos do not need external drivers. The DC motors have quadrature encoders. There are also 10 limit switches for m1 to m4 as well as m6.

#### Wiring While Connected to Rover
On the rover, there is a dedicated PCB that the Teensy plugs into. There are 5 cables which connect the arm to the electronics bay and therefore to the arm PCB: one for power to the 4 DC motors, two for the DC motor encoders, one for the two servos, and one for all the limit switches.

#### Wiring While NOT Connected to Rover (OLD)
For prototyping purposes: there is a .docx file in `/robot/rover/Arm` which shows wiring diagrams for the different drivers and steppers FROM A PREVIOUS DESIGN ITERATION. Note that in all cases but the servos, the power supply ground and the logic-level ground are independent. In the case of the servos, the ground will be isolated in the final design, but for testing purposes it can share a common ground. The 8-wire stepper motor coils are in series configuration.

In short, the Teensy is connected to the drivers (and the servos). The drivers (and the servos) are powered by power supplies (7.4V for the servos, 12V for the DC motors and steppers) and connect to the motors (aside from the servos).

### Doxygen
This folder contains files related to the Doxygen auto-generated documentation.
"Doxyfile" is what my laptop uses to generate the documentation, including where on my laptop to place it.
"index.html - Shortcut" links to the most recently generated documentation on my laptop.
"ArmDriverUnit- Main Page" is a link to the most recently uploaded version of the documentation to my personal ENCS website.

### Examples
This folder contains example sketches for using the classes built in ArmDriverUnit. In the future, once the classes are turned into actual libraries, these sketches will come with the library.

Since the classes are still in development, the code structure gets changed quite frequently, necessitating updates to these example files too. It is likely that they are not as up to date as ArmDriverUnit.

#### ServoExampleOpenLoop
This example demonstrates open loop control of a single servo. It's a good introduction into the structure and logic of the code, but is lacking many features.

#### DcExampleTwoMotors
This example demonstrates open or closed loop of two DC motors. It's the next step in understanding how multiple motors are manipulated simultaneously. Once this can be understood, you can probably figure out how to code it for all 3 types of motors running at once.

#### RosCommunication
This example demonstrates how the Teensy can communicate with ROS over the serial port. It hasn't been tested yet but you should be able to choose either USB serial or hardware serial. Please note that you will need to modify one of the rosserial header files to make it work. Instructions are in `robotics-prototype/README.md`.

## MobilePlatform
### Code Structure

The Mobile Platform code follows a similar structure to the Arm code. It still uses RobotMotor and DcMotor, however it implements its own class for handling commands. It is capable of being controlled via USB, UART or bluetooth.

### Hardware and Wiring

The current iteration of the mobile platform has 6 DC motors, all controlled by Cytron drivers. It also has two position servos and two continuous servos for aiming the front and rear cameras. Finally, it has a GPS module, an IMU, and a connector for a bluetooth module.

#### Wiring While Connected to Rover

On the rover, there is a dedicated PCB that the Teensy, GPS module and IMU plug into, and optionally a bluetooth module as well. There are 5 cables which connect the mobile platform to the electronics bay and therefore to the mobile platform PCB: one for power to the 6 DC motors, two for the DC motor encoders, and two for the four servos. There is also a connector on the electronics bay for an extension to the GPS antenna which is fixed to the top of the electronics bay. 

## Science


## PDS


## Demos
This folder contains short demo sketches to test simple Arduino/Teensy features.

### EncoderTest
This is a very simple Teensy 3.6 sketch which prints "beep" and "boop" depending on which encoder channel triggers an interrupt on the microcontroller. It is used simply to demonstrate that the interrupt code works before moving to more advanced code.

### RoverControlDemo
This Teensy 3.6 sketch listens for 'w' and 's' over the Serial port and responds by controlling the speed of two motors.

### SimpleServoTest
This Teensy 3.6 sketch was created after the discovery that attempting to stop a continuous servo using analogWrite with a PWM signal of 127 did not work but 189 did. After testing it was discovered that using the Servo library and the time in microseconds, the servo can be controlled as expected. 1500 for stopping the servo and 1000 or 2000 for max speed in either direction.

## Utilities
This folder contains sketches used for testing or measuring data. Due to their nature, some may use older code and may require refactoring in the future.

### Servo_Temp_Test
This example makes one or two servos turn at max speed in a certain direction. This allows someone to test how hot they get when they run continuously for several minutes.

### Stepper_Temp_Test
This example makes one or two steppers hold their position or turn in a certain direction. This allows someone to test how hot they get when they are drawing current continuously for several minutes.

### MotorCharacterization
This is a Teensy 3.6 sketch which prints the step response of a DC motor over the serial port.

It expects a step input and uses encoder interrupts to fill in an array of durations representing the time between each interrupt. After the motor has reached steady state (currently the user has to guess how long this will take), the duration array is used to calculate an array of speeds in the units desired by the user and prints the values over the serial port.

The result can be used to develop a model for the motor and then to design a PID for it.

## TestCode
This directory contains test code which may or may not go into the main code being developed.

## PidController
This is an older project that is currently going unused.

NOTE: the Adafruit library most probably won't end up being used since we decided against using adafruit motorshields in the end,
although this sketch will be either heavily refactored or completely rewritten by operations and power squads for 3 stepper, 1 DC
and 2 servo motors, this sketch can still be used to test interfacing python/arduino using a PID system to control a more simpler
set of motors, such as positional (non-continous) servos.
The moving of motors and current sensor readings will be programmed by the power squad.
Both teams need to decide on common expected formats for data to be properly communicated over serial.

To obtain the libraries using Arduino IDE you can click on `sketch` > `include library` > `manage libraries` and search to find them.
Simply searching for 'adafruit' and 'encoder' and choosing the most appropriate looking libraries from the list is all you need to do.
This method installs it in your arduino libraries folder.

If you wish to refer the `include` statements to set your relative path to the libraries, you must surround the libary name with double quotes.

## test.py

Simple but useful python script for quickly testing serial comms.

## StreamDispatcher.py
Currently this starts a stream via `start_stream.sh` which uses mjpgstreamer, more details are documented in the bash script itself.
To start the stream, assuming you already have a usb web camera connected, run `./StreamDispatcher.py`. Use `Ctrl + C` to stop the stream.

Assuming the stream is being dispatched, it can be accessed in any browser by visiting: `server_ip:8090/?action=stream`,
where `server_ip` is replaced with the actual ip address  of the server doing the streaming.
If you have started the StreamDispatcher, opening the Arm page will show the live of feed of the camera in the Arm Vision panel.
The rover ip address in `app.py` will have to be set to the correct value.

Example:
`#include "libraries/Encoder.h"`

### Capturing the stream to mp4

```
sudo apt install ffmpeg
ffmpeg -i "http://127.0.0.1:8090/?action=stream" -c:v libx264 -preset veryslow -crf 18 output.mp4
```

- `-crf 18` --> 6000 kb / s bitrate, excellent quality
- `-crf 30` --> 500 kb / s bitrate, medium quality
