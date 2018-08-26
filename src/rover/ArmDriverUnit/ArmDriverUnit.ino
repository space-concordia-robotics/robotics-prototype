/*
  TODO:
  -replace strtok with strtok_r when implementing interrupts

  -for motor control, budge time is in seconds but this isn't very granular.
   milliseconds is ideal but requires different parsing method for time length.
  
  -implement simple manual control:
   -import stepper library and maybe DC library for simple manual control
   -use libraries to write functions (that aren't optimized) for manual control
   -rewrite parsing function to parts what tatum will be sending (x,y)
  
  -fix issue where i need to create the motor objects inside the motordriver header
   -fixed: the issue is a circular dependency solved by placing the encoder count variables elsewhere
  
  -implement appropriate timer ISRs that can take input from PID or manual control functions:
   -rewrite all the register bit variables to use teensy registers for encoder/limit switch interrupts
   -rewrite the motor control with timers for teensy, ensure control for all motors
   
  -implement PID for automatic control
	-update parsing function or just go directly to ROSserial
	
  -incorporate limit switches for homing position on all motors
  -implement homing function on boot

  -timers
   -systick is normally a heartbeat type thing
   -lptmr runs even on low power mode, maybe this should be heartbeat instead?
   -pit is used for intervaltimer objects, there are 4 and they work like interrupts
    -decide frequency of motor control loop: figure out estimated time for loop
   -pwm: teensy has 6 16bit pwm timers and apparently 22 total pwm options:
    -
   -teensy pwm page mentions ftm and tpm timers which aren't mentioned in the page with other timesr
   -ftm+tpm has quadrature decoder?
   -tpm is 2-8 channel timer with pwm, 16bit counter, 2 channels for pwm
   
  -implement error checking?
  -implement power management? sleep mode?
 */

#include "PinSetup.h"
#include "ArmMotor.h"

#define BAUD_RATE 115200    //serial baud rate
#define BUFFER_SIZE 100     //size of the buffer for the serial commands

char serialBugger[BUFFER_SIZE];

int whichMotor; int whichDir; int whichSpeed; int tempSpeedVar; // int whichTime; int tempTimeVar;


ArmMotor motor1(MOTOR1); // base stepper (rotation)
ArmMotor motor2(MOTOR2); // shoulder dc (flexion)
ArmMotor motor3(MOTOR3); // elbow stepper (flexion)
ArmMotor motor4(MOTOR4); // wrist stepper (flexion)
ArmMotor motor5(MOTOR5); // wrist servo (rotation)
ArmMotor motor6(MOTOR6); // end effector servo (pinching)

void setup() {
  pinSetup();
  Serial.begin(BAUD_RATE);
}

void loop() {
  
  //digitalWrite(LED_PIN, led_status);

   if (Serial.available()) { // make sure the whole message was received
    Serial.readBytesUntil(10, serialBuffer, 100); //read until NL
    Serial.print("GOT: ");
    Serial.println(serialBuffer);
	
    //parseCommand(serialBuffer);           //parse the string
	
	/* 
	 * instead of using the parse function i dumped the code here for now.
	 * This code figures out which motor and which direction, code further
	 * down actually tells the motors to move
	 */
	
	//strtok_r() splits message by a delimiter string
	char* msgElem = strtok(serialBuffer, " "); // look for first element (first tag)
	if (String(msgElem) == "motor"){
		msgElem = strtok(NULL, " "); // go to next msg element (motor number)
		switch(*msgElem){ // determines which motor being used
			case '1':
				whichMotor = MOTOR1;
				break;
			case '2':
				whichMotor = MOTOR2;
				break;
			case '3':
				whichMotor = MOTOR3;
				break;
			case '4':
				whichMotor = MOTOR4;
				break;
			case '5':
				whichMotor = MOTOR5;
				break;
			case '6':
				whichMotor = MOTOR6;
				break;
		}
		Serial.print("motor"); Serial.println(whichMotor);
		msgElem = strtok(NULL, " "); //find the next message element (direction tag)
		if (String(msgElem) == "direction"){
			msgElem = strtok(NULL, " "); // go to next msg element (direction)
			switch(*msgElem){ // determines motor direction
				case '0':
					whichDir = CLOCKWISE;
					Serial.println("clockwise");
					break;
				case '1':
					whichDir = COUNTER_CLOCKWISE;
					Serial.println("counter-clockwise");
					break;
			}
			msgElem = strtok(NULL, " "); // find the next message element (time tag)
			if (String(msgElem) == "speed"){
			    msgElem = strok(NULL, " "); // find the next message element (integer representing speed level)
				tempSpeedVar = *msgElem - '0'; // example '3' - '0' = 3
				if (tempSpeedVar <= 0){ // make sure the subtraction made sense, if it's above 9 it doesn't
					whichSpeed = tempSpeedVar;
					Serialprint("speed level: "); Serial.println(whichSpeed);
				}
			}
			/*msgElem = strtok(NULL, " "); //find the next message element (time tag)
			if (*msgElem == 'time'){
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
			motor1.sinceStart = 0;
			motor1.budge(whichDir);
			break;
		case MOTOR2:
			motor2.sinceStart = 0;
			while (motor2.sinceStart < BUDGE_TIME) {
				motor2.budge(whichDir);
			}
			break;
		case MOTOR3:
			motor3.sinceStart = 0;
			motor3.budge(whichDir);
			break;
		case MOTOR4:
			motor4.sinceStart = 0;
			motor4.budge(whichDir);
			break;
		case MOTOR5:
			motor5.budge(whichDir);
			break;
		case MOTOR6:
			motor6.sinceStart = 0;
			while (motor6.sinceStart < BUDGE_TIME) {
				motor6.budge(whichDir);
			}
			break;
	}
whichMotor = 0;

}

