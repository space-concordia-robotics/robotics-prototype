# Classes

The Teensy code for the Arm is abstracted into classes to aid in code modularity. Currently there is the RobotMotor class with the DcMotor, StepperMotor (not used anymore) and ServoMotor subclasses. There are also the PidController and Parser classes, and pinSetup which is not a class but is separated into its own .cpp and .h files. In the future, these classes could be turned into Arduino libraries. At the very least, a similar structure can be used on the rover Teensy.

# Header files

Includes.h holds many of the macros and include directives so that the code in Arm.ino is less bloated
Vsense.h holds battery voltage sensing functionality
Notes.h and Ideas.h hold sample code and ideas for future improvements.

# Hardware and Wiring

The current iteration of the rover arm has m1 to m4 being DC motors and m5 and m6 being continuous rotation servos. Currently the DC motors are driven with Cytron drivers and the servos do not need external drivers. The DC motors have quadrature encoders. There are also 10 limit switches for m1 to m4 as well as m6.

## Wiring While Connected to Rover
On the rover, there is a dedicated PCB that the Teensy plugs into. There are 5 cables which connect the arm to the electronics bay and therefore to the arm PCB: one for power to the 4 DC motors, two for the DC motor encoders, one for the two servos, and one for all the limit switches.

## Wiring While NOT Connected to Rover (OLD)
For prototyping purposes: there is a .docx file in `/robot/rover/Arm` which shows wiring diagrams for the different drivers and steppers FROM A PREVIOUS DESIGN ITERATION. Note that in all cases but the servos, the power supply ground and the logic-level ground are independent. In the case of the servos, the ground will be isolated in the final design, but for testing purposes it can share a common ground. The 8-wire stepper motor coils are in series configuration.

In short, the Teensy is connected to the drivers (and the servos). The drivers (and the servos) are powered by power supplies (7.4V for the servos, 12V for the DC motors and steppers) and connect to the motors (aside from the servos).

# Doxygen
This folder contains files related to the Doxygen auto-generated documentation.
"Doxyfile" is what my laptop uses to generate the documentation, including where on my laptop to place it.
"index.html - Shortcut" links to the most recently generated documentation on my laptop.
"ArmDriverUnit- Main Page" is a link to the most recently uploaded version of the documentation to my personal ENCS website.

# Examples
This folder contains example sketches for using the classes built in ArmDriverUnit. In the future, once the classes are turned into actual libraries, these sketches will come with the library.

Since the classes are still in development, the code structure gets changed quite frequently, necessitating updates to these example files too. It is likely that they are not as up to date as ArmDriverUnit.

#### ServoExampleOpenLoop
This example demonstrates open loop control of a single servo. It's a good introduction into the structure and logic of the code, but is lacking many features.

#### DcExampleTwoMotors
This example demonstrates open or closed loop of two DC motors. It's the next step in understanding how multiple motors are manipulated simultaneously. Once this can be understood, you can probably figure out how to code it for all 3 types of motors running at once.

#### RosCommunication
This example demonstrates how the Teensy can communicate with ROS over the serial port. It hasn't been tested yet but you should be able to choose either USB serial or hardware serial. Please note that you will need to modify one of the rosserial header files to make it work. Instructions are in `robotics-prototype/README.md`.
