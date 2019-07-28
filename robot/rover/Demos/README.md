# EncoderTest
This is a very simple Teensy 3.6 sketch which prints "beep" and "boop" depending on which encoder channel triggers an interrupt on the microcontroller. It is used simply to demonstrate that the interrupt code works before moving to more advanced code.

# RoverControlDemo
This Teensy 3.6 sketch listens for 'w' and 's' over the Serial port and responds by controlling the speed of two motors.

# SimpleServoTest
This Teensy 3.6 sketch was created after the discovery that attempting to stop a continuous servo using analogWrite with a PWM signal of 127 did not work but 189 did. After testing it was discovered that using the Servo library and the time in microseconds, the servo can be controlled as expected. 1500 for stopping the servo and 1000 or 2000 for max speed in either direction.
