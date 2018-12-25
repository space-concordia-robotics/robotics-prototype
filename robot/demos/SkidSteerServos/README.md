# Skid Steering Demo

## Summary:
- Proof of concept for Skid steering function.
- Tested on Continuous rotation servos “Blue Boys”
- Tested on a small 3D printed prototype consisting of a 3D printed body and wheels, six servos, Bluetooth module, LiPo Batteries and an ArduinoCode Uno, which was controlled by an app named Arduino Blue. Test result were successful.
- Second code file is configured for Dc motors
- Test on Dc Motor is still required for final validation.

## Results
Test was successful but requires further testing for DC motors. 

## Setup

### Hardware (TEST)
- 3D printed Body and six Wheels of a Miniature Rover Replica.
- Six Continuous Servoce model FS90R (BlueBoys).
- HM10 bluetooth module (Not authentic version).
- Two 3.7V 500MAH LiPo Batteries
- Arduino Uno

### Pin Assemply
#### All Servos
Red | 5V
------------ | -------------
Brown | Ground

#### Front Right Servo
Yellow | PIN 9
------------ | -------------

#### Middle Right Servo
Yellow | PIN 10
------------ | -------------

#### Back Right Servo
Yellow | PIN 11
------------ | -------------

#### Front Left Servo
Yellow | PIN 3
------------ | -------------

#### Middle Left Servo
Yellow | PIN 5
------------ | -------------

#### Back Left Servo
Yellow | PIN 6
------------ | -------------


#### HM10 bluetooth module
VCC | 5V
------------ | -------------
GND | Ground
TXD | PIN 7
RXD | PIN 8

#### Batteries are Connected in Series 
Positive | Vin
------------ | -------------
Negative | Ground

### Software
You will need to import the library ArduinoBlue for bluetooth connection with android/iOS. You may use the Arduino IDE library manager: 

- Navigate to: Sketch > Include Library > Manage Libraries...
- Search for 'arduinoblue'
- Install

Download Arduino App on your smart phone. From there you can easily connect to your mini rover and control it from the drive function in the app.
