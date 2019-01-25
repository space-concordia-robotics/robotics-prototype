# robotics-prototype\robot\rover
This repo contains the code/documentation going into the rover.

## ArmDriverUnit (ADU)
### Code Structure

The Teensy code for the ADU is abstracted into classes to aid in code modularity. Currently there is the RobotMotor class with the DcMotor, StepperMotor and ServoMotor subclasses. There are also the PidController and Parser classes, and pinSetup which is not a class but is separated into its own .cpp and .h files. In the future, these classes could be turned into Arduino libraries. At the very least, a similar structure can be used on the rover Teensy.

### Hardware and Wiring

The current iteration of the rover arm has m1 and m2 being DC motors, m3 and m4 being stepper motors, and m5 and m6 being continuous rotation servos. Currently the DC motors are driven with Cytron drivers, the stepper motors with A4988 or DRV8825 drivers, and the servos do not need external drivers.

For prototyping purposes: there is a .docx file in `/robot/rover/ArmDriverUnit` which shows wiring diagrams for the different drivers and steppers. Note that in all cases but the servos, the power supply ground and the logic-level ground are independent. In the case of the servos, the ground will be isolated in the final design, but for testing purposes it can share a common ground. The 8-wire stepper motor coils are in series configuration.

In short, the Teensy is connected to the drivers (and the servos). The drivers (and the servos) are powered by power supplies (7.4V for the servos, 12V for the DC motors and steppers) and connect to the motors (aside from the servos).

On the rover, there will be a dedicated board with (probably) a d-sub connector that the Teensy plugs into. A cable plugged into the d-sub connector will branch out and connect to the respective motors and driver circuits.

## Examples
This folder contains example sketches for using the classes built in ArmDriverUnit. In the future, once the classes are turned into actual libraries, these sketches will come with the library.

Since the classes are still in development, the code structure gets changed quite frequently, necessitating updates to these example files too. It is likely that they are not as up to date as ArmDriverUnit.

## RoverDriverUnit (RDU)
### Code Structure

The RDU code structure is to be decided but will probably follow a similar structure to the ADU.

### Hardware and Wiring

The current iteration of the rover has 6 DC motors, all controlled by Cytron drivers.

## EncoderTest
This is a very simple Teensy 3.6 sketch which prints "beep" and "boop" depending on which encoder channel triggers an interrupt on the microcontroller. It is used simply to demonstrate that the interrupt code works before moving to more advanced code.

## MotorCharacterization
This is a Teensy 3.6 sketch which prints the step response of a DC motor over the serial port.

It expects a step input and uses encoder interrupts to fill in an array of durations representing the time between each interrupt. After the motor has reached steady state (currently the user has to guess how long this will take), the duration array is used to calculate an array of speeds in the units desired by the user and prints the values over the serial port.

The result can be used to develop a model for the motor and then to design a PID for it.

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

Example:
`#include "libraries/Encoder.h"`

## TestCode
This directory contains test code which may or may not go into ArmDriverUnit.