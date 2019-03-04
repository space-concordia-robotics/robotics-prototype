# robotics-prototype\robot\rover\Examples
This repo contains examples of how to implement the files that have been developed. It may not always be up to date with the final version of the code, as the code is still in development and hasn't been finalized into libraries yet.

##ServoExampleOpenLoop
This example demonstrates open loop control of a single servo. It's a good introduction into the structure and logic of the code, but is lacking many features.

## DcExampleTwoMotors
This example demonstrates open or closed loop of two DC motors. It's the next step in understanding how multiple motors are manipulated simultaneously. Once this can be understood, you can probably figure out how to code it for all 3 types of motors running at once.

##Servo_Temp_Test
This example makes one or two servos turn at max speed in a certain direction. This allows someone to test how hot they get when they run continuously for several minutes.

##Stepper_Temp_Test
This example makes one or two steppers hold their position or turn in a certain direction. This allows someone to test how hot they get when they are drawing current continuously for several minutes.

## RosCommunication
This example demonstrates how the Teensy can communicate with ROS over the serial port. It hasn't been tested yet but you should be able to choose either USB serial or hardware serial. Please note that you will need to modify one of the rosserial header files to make it work. Instructions are in `robotics-prototype/README.md`.