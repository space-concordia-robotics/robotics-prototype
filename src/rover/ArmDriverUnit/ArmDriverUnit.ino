/*
  TODO:
  -make sure there is  max/min angle limitation, either with encoders or with default speed/time and counting buttonclicks
  -suggesgtion 1 pinmode input, 2 is micros pwm instead of analogwrite (timer is bad?)
  -(always) implement error checking, remove parts later on if it's too slow (tatum doubts)
  -(always) clean up code
  -(later) implement power management? sleep mode?
  -(later) implement PID for automatic control
    -update parsing function to take in angles or just go directly to ROSserial, probably the former
  -(done-ish) fix issue where i need to create the motor objects inside the motordriver header
  -(next) solve many problems by creating motor type subclasses
  
  -(priority) implement simple manual control:
   -(done) parsing procedure can process what tatum sends
   -(done) servo and dc motor control work and behave the same way
   -(done) direction, speed and time are all options and they have default values
   -(done) leave serial.print debugging messages unless it really takes too much time (doubt)
   -(done) parsing code is in an object, may revert later
   -(probably done, need to test)figure out how to solve the servo jitter. setting pins to INPUT probably fixes it
   -(done, need abtin's circuit, but test with 3.3 for now) figure out how to step up to 5v for the sabertooth
   -(now) connect and test stepper and stepper driver with Teensy code
   -instead of setting whichMotor to 0, perform some kind of check so that the motor doesn't spin forever
   -figure out how to send motor info from parsing function to budgeMotor etc (move motor object creation into setup() )
   -determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
   
   -(next) replace strtok with strtok_r when implementing interrupts
   -(next step) decide whether to use abtin's software interrupt or simpler method for simultaneous motor control
    -implement appropriate software interrupts that can take input from PID or manual control functions?
    -decide frequency of motor control loop: figure out estimated time for loop
    -otherwise figure out a simpler way of doing it
   -implement heartbeat after deciding on best timer for it (see below)
    
  -(next next step) external interrupts for encoders and limit switches
   -determine whether it's worth it to use the built in quadrature decoders
   -rewrite all the register bit variables to use teensy registers for encoder/limit switch interrupts
   -rewrite the motor control with timers for teensy, ensure control for all motors
   -incorporate limit switches for homing position on all motors
   -implement homing function on boot
   -(next) import stepper library for simple manual control or write fancy code
   
  -(next step) timers
   -systick is normally a heartbeat type thing
   -lptmr runs even on low power mode, maybe this should be heartbeat instead?
   -pit is used for intervaltimer objects, there are 4 and they work like interrupts
   -pwm: teensy has 6 16bit pwm timers and apparently 22 total pwm options:
    -currently using whatever is connected to the pwm pins for timing pwm
    -teensy pwm page mentions ftm and tpm timers which aren't mentioned in the page with other timesr
    -ftm+tpm has quadrature decoder?
    -tpm is 2-8 channel timer with pwm, 16bit counter, 2 channels for pwm
 */

#include "PinSetup.h"
#include "ArmMotor.h"
#include "SerialParser.h"

SerialParser serialParser;

// these objects were created outside of the setup function because the update() proto-code isn't in the right place yet
ArmMotor motor1(MOTOR1); // base stepper (rotation)
ArmMotor motor2(MOTOR2); // shoulder dc (flexion)
ArmMotor motor3(MOTOR3); // elbow stepper (flexion)
ArmMotor motor4(MOTOR4); // wrist stepper (flexion)
ArmMotor motor5(MOTOR5); // wrist servo (rotation)
ArmMotor motor6(MOTOR6); // end effector servo (pinching)
  
void setup() {
  pinSetup();
  
  // place armmotor object creations here once code is cleaned up
}

void loop() {
  
  //digitalWrite(LED_PIN, led_status);

	serialParser.parseSerial();

  int whichMotor = serialParser.whichMotor;
	int whichDir = serialParser.whichDir;
	int whichSpeed = serialParser.whichSpeed;
	unsigned int whichTime = serialParser.whichTime;
	
	//proto code for ArmMotor::update() function?
	switch(whichMotor){ // tells the appropriate motor to move with ArmMotor.budge()
		case MOTOR1:
			motor1.budge(whichDir,whichSpeed,whichTime);
			break;
		case MOTOR2:
			motor2.budge(whichDir,whichSpeed,whichTime);
			break;
		case MOTOR3:
			motor3.budge(whichDir,whichSpeed,whichTime);
			break;
		case MOTOR4:
			motor4.budge(whichDir,whichSpeed,whichTime);
			break;
		case MOTOR5:
			motor5.budge(whichDir,whichSpeed,whichTime);
			break;
		case MOTOR6:
			motor6.budge(whichDir,whichSpeed,whichTime);
			break;
	}
 whichMotor=0; // this was a quick fix so that at the next loop it will wait for a new message
  
}


