# Code Structure

The Mobile Platform code follows a similar structure to the Arm code. It still uses RobotMotor and DcMotor, however it implements its own class for handling commands. It is capable of being controlled via USB, UART or bluetooth.

# Hardware and Wiring

The current iteration of the mobile platform has 6 DC motors, all controlled by Cytron drivers. It also has two position servos and two continuous servos for aiming the front and rear cameras. Finally, it has a GPS module, an IMU, and a connector for a bluetooth module.

## Wiring While Connected to Rover

On the rover, there is a dedicated PCB that the Teensy, GPS module and IMU plug into, and optionally a bluetooth module as well. There are 5 cables which connect the mobile platform to the electronics bay and therefore to the mobile platform PCB: one for power to the 6 DC motors, two for the DC motor encoders, and two for the four servos. There is also a connector on the electronics bay for an extension to the GPS antenna which is fixed to the top of the electronics bay. 
