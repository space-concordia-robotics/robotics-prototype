# Servo_Temp_Test
This example makes one or two servos turn at max speed in a certain direction. This allows someone to test how hot they get when they run continuously for several minutes.

# Stepper_Temp_Test
This example makes one or two steppers hold their position or turn in a certain direction. This allows someone to test how hot they get when they are drawing current continuously for several minutes.

# MotorCharacterization
This is a Teensy 3.6 sketch which prints the step response of a DC motor over the serial port.

It expects a step input and uses encoder interrupts to fill in an array of durations representing the time between each interrupt. After the motor has reached steady state (currently the user has to guess how long this will take), the duration array is used to calculate an array of speeds in the units desired by the user and prints the values over the serial port.

The result can be used to develop a model for the motor and then to design a PID for it.
