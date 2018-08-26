/*
  TODO:
  -implement simple manual control:
   -(done) parsing procedure can process what tatum sends
   -(done) servo and dc motor control work and behave the same way
   
   -put parsing code into an actual function (move variables out of this file)
   -instead of seting whichMotor to 0, perform some kind of check so that the motor doesn't spin forever
   -figure out how to send motor info from parsing function to budgeMotor etc (move motor object creation into setup() )
   
   -import stepper library for simple manual control
   -replace strtok with strtok_r when implementing interrupts
   -connect and test stepper and stepper driver with Teensy code
   
   -determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
   -comment out serial.print debugging messages? maybe not
   
   -decide whether to use abtin's software interrupt or simpler method for simultaneous motor control
    -implement appropriate software interrupts that can take input from PID or manual control functions?
    -decide frequency of motor control loop: figure out estimated time for loop
    -otherwise figure out a simpler way of doing it
   -implement heartbeat after deciding on best timer for it (see below)
   
   -implement speed control or time control, decide what value to fix the one not being controlled
   -figure out how to solve the servo jitter
   -figure out how to step up to 5v for the sabertooth
     
-fix issue where i need to create the motor objects inside the motordriver header
   -(done): the issue is a circular dependency solved by placing the encoder count variables elsewhere
  
  -external interrupts for encoders and limit switches
   -determine whether it's worth it to use the built in quadrature decoders
   -rewrite all the register bit variables to use teensy registers for encoder/limit switch interrupts
   -rewrite the motor control with timers for teensy, ensure control for all motors
  -incorporate limit switches for homing position on all motors
  -implement homing function on boot
   
  -implement PID for automatic control
	-update parsing function to take in angles or just go directly to ROSserial, probably the former
	
  -timers
   -systick is normally a heartbeat type thing
   -lptmr runs even on low power mode, maybe this should be heartbeat instead?
   -pit is used for intervaltimer objects, there are 4 and they work like interrupts
   -pwm: teensy has 6 16bit pwm timers and apparently 22 total pwm options:
    -currently using whatever is connected to the pwm pins for timing pwm
    -teensy pwm page mentions ftm and tpm timers which aren't mentioned in the page with other timesr
    -ftm+tpm has quadrature decoder?
    -tpm is 2-8 channel timer with pwm, 16bit counter, 2 channels for pwm
   
  -implement error checking?
  -implement power management? sleep mode?
 */

#include "PinSetup.h"
#include "ArmMotor.h"

// definitions and variables for parsing function: to put in self-contained header file
#define BAUD_RATE 115200 // serial baud rate
#define BUFFER_SIZE 100  // size of the buffer for the serial commands
char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
int whichMotor;
int whichDir;
int whichSpeed; int tempSpeedVar;
// int whichTime; int tempTimeVar;

// these objects were created outside of the setup function because the update() proto-code isn't the right place yet
ArmMotor motor1(MOTOR1); // base stepper (rotation)
ArmMotor motor2(MOTOR2); // shoulder dc (flexion)
ArmMotor motor3(MOTOR3); // elbow stepper (flexion)
ArmMotor motor4(MOTOR4); // wrist stepper (flexion)
ArmMotor motor5(MOTOR5); // wrist servo (rotation)
ArmMotor motor6(MOTOR6); // end effector servo (pinching)

void setup() {
  pinSetup();
  Serial.begin(BAUD_RATE);
  // place armmotor object creations here once code is cleaned up
}

void loop() {
  
  //digitalWrite(LED_PIN, led_status);

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received

    //parseCommand(serialBuffer);           //parse the string
	
	/* 
	 * instead of using the parse function i dumped the code here for now.
	 * This code figures out which motor and which direction, code further
	 * down actually tells the motors to move
	 */
	
	//strtok_r() splits message by a delimiter string
	char* msgElem = strtok(serialBuffer, " "); // look for first element (first tag)
	if (String(msgElem) == "motor"){ // msgElem is a char array so it's safer to convert to string first
		msgElem = strtok(NULL, " "); // go to next msg element (motor number)
		switch(*msgElem){ // determines which motor being used
			case '1':
				whichMotor = MOTOR1;
				Serial.print("motor 1");
				break;
			case '2':
				whichMotor = MOTOR2;
				Serial.print("motor 2");
				break;
			case '3':
				whichMotor = MOTOR3;
				Serial.print("motor 3");
				break;
			case '4':
				whichMotor = MOTOR4;
				Serial.print("motor 4");
				break;
			case '5':
				whichMotor = MOTOR5;
				Serial.print("motor 5");
				break;
			case '6':
				whichMotor = MOTOR6;
				Serial.print("motor 6");
				break;
		}
		Serial.print("motor"); Serial.println(whichMotor);
		msgElem = strtok(NULL, " "); //find the next message element (direction tag)
		if (String(msgElem) == "direction"){ // msgElem is a char array so it's safer to convert to string first
			msgElem = strtok(NULL, " "); // go to next msg element (direction)
			switch(*msgElem){ // determines motor direction
				case '0': // arbitrarily (for now) decided 0 is clockwise
					whichDir = CLOCKWISE;
					Serial.println("clockwise");
					break;
				case '1': // arbitrarily (for now) decided 1 is counter-clockwise
					whichDir = COUNTER_CLOCKWISE;
					Serial.println("counter-clockwise");
					break;
			}
			msgElem = strtok(NULL, " "); // find the next message element (time tag)
			if (String(msgElem) == "speed"){ // msgElem is a char array so it's safer to convert to string first
			    msgElem = strok(NULL, " "); // find the next message element (integer representing speed level)
				tempSpeedVar = *msgElem - '0'; // example '3' - '0' = 3
				if (tempSpeedVar <= 9){ // make sure the subtraction made sense. If it's above 9, it doesn't
					whichSpeed = tempSpeedVar; // set the actual speed
					Serialprint("speed level: "); Serial.println(whichSpeed);
				}
			}
			/*msgElem = strtok(NULL, " "); //find the next message element (time tag)
			if (*msgElem == 'time'){ // msgElem is a char array so it's safer to convert to string first
				msgElem = strtok(NULL, " "); //find the next message element (time in seconds)
				// because of how this works it only takes 0-3 sec, must rework later
				tempTimeVar = *msgElem - '0'; // example: '3' - '0' = 3
				if (tempTimeVar <= MAX_BUDGE_TIME){ // don't allow budge movements to last a long time
					whichTime = tempTimeVar;
					Serial.print(whichTime); Serial.println("s");
				}
			}*/
		}
	}
	memset(serialBuffer, 0, BUFFER_SIZE); //empty the buffer
  }
	//proto code for ArmMotor::update() function?
	switch(whichMotor){ // tells the appropriate motor to move with ArmMotor.budge()
		case MOTOR1:
			motor1.budge(whichDir);
			break;
		case MOTOR2:
			motor2.budge(whichDir);
			break;
		case MOTOR3:
			motor3.budge(whichDir);
			break;
		case MOTOR4:
			motor4.budge(whichDir);
			break;
		case MOTOR5:
			motor5.budge(whichDir);
			break;
		case MOTOR6:
			motor6.budge(whichDir);
			break;
	}
whichMotor = 0; // this was a quick fix so that at the next loop it will wait for a new message

}

