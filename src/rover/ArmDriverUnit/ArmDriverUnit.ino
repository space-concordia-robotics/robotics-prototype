/*
  TODO:
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

void parseCommand(char* line);

void setup() {
  pinSetup();
  Serial.begin(BAUD_RATE);
  
  ArmMotor motor1(MOTOR1); // base stepper (rotation)
  ArmMotor motor2(MOTOR2); // shoulder dc (flexion)
  ArmMotor motor3(MOTOR3); // elbow stepper (flexion)
  ArmMotor motor4(MOTOR4); // wrist stepper (flexion)
  ArmMotor motor5(MOTOR5); // wrist servo (rotation)
  ArmMotor motor6(MOTOR6); // end effector servo (pinching)

  /*
  steppers[0].dirFunc  = M1Dir;
  steppers[0].stepFunc = M1Step;

  steppers[1].dirFunc  = M2Dir;
  steppers[1].stepFunc = M2Step;

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1000;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= ((1 << CS11) | (1 << CS10));
  interrupts();

  //start of the heartbeat timer
  startTime = millis();
  */
}

/* void ISR(TIMER1_COMPA_vect)
{
  for (int i = 0; i < NUM_STEPPERS; i++) {
    volatile stepperInfo& s = steppers[i];

    if (s.currentAngle >= 360) {
      s.currentAngle = 0;
    }
    if (s.currentAngle < 0) {
      s.currentAngle = 360;
    }

    if ( abs(s.currentAngle - s.desiredAngle) > 1.7 && !s.movementDone  ) {
      s.stepFunc();
      s.currentAngle += s.dir;

    } else {
      s.movementDone = true;
    }

    if (steppers[0].movementDone && steppers[1].movementDone) {  //very dirty. we dont want to check manually.
      TIMER1_INTERRUPTS_OFF
    }
  }
}
*/
/* void prepareMovement(int stepperNumber, float angle) {

  volatile stepperInfo& s = steppers[stepperNumber];

  s.desiredAngle = angle;
  s.movementDone = false;

  if (s.currentAngle < s.desiredAngle) {
    if (abs(s.currentAngle - s.desiredAngle) < 180) {
      s.dir = +1.8;
      s.dirFunc(1);
    } else {
      s.dir = -1.8;
      s.dirFunc(0);
    }
  } else {
    if (abs(s.currentAngle - s.desiredAngle) < 180) {
      s.dir = -1.8;
      s.dirFunc(0);
    } else {
      s.dir = +1.8;
      s.dirFunc(1);
    }
  }
}
*/
/* void startMovement() {
  OCR1A = c0;
  TIMER1_INTERRUPTS_ON
}
*/

/* void prepareDCMotor(float angle) {

   dcMotor.desiredAngle = angle;
  dcMotor.dcMovementDone = false;

   if (dcMotor.currentAngle < dcMotor.desiredAngle) {
    if (abs(dcMotor.currentAngle - dcMotor.desiredAngle) < 180) {
      dcMotor.dir = 100;
    } else {
      dcMotor.dir = 150;
    }
  } else {
    if (abs(dcMotor.currentAngle - dcMotor.desiredAngle) < 180) {
      dcMotor.dir = 150;
    } else {
      dcMotor.dir = 100;
    }
  }
}
*/

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
	
	//strtok() splits message by a delimiter string
	char* msgElem = strtok(serialBuffer, " "); // look for first element (first tag)
	if (*msgElem == 'motor'){
		int whichMotor; int whichDir; int whichTime; int tempTimeVar;
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
			default:
				Serial.print("motor"); Serial.println(whichMotor);
		}
		msgElem = strtok(NULL, " "); //find the next message element (direction tag)
		if (*msgElem == 'direction'){
			msgElem = strtok(NULL, " "); // go to next msg element (direction)
			switch(*msgElem){ // determines motor direction
				case 'cw':
					whichDir = CLOCKWISE;
					Serial.println("clockwise");
					break;
				case 'ccw':
					whichDir = COUNTER_CLOCKWISE;
					Serial.println("counter-clockwise");
					break;
			}
			msgElem = strtok(NULL, " "); //find the next message element (time tag)
			if (*msgElem == 'time'){
				msgElem = strtok(NULL, " "); //find the next message element (time in seconds)
				/* because of how this works it only takes 0-3 sec, must rework later */
				tempTimeVar = *msgElem - '0'; // example: '3' - '0' = 3
				if (tempTimeVar <= MAX_BUDGE_TIME){ // don't allow budge movements to last a long time
					whichTime = tempTimeVar;
					Serial.print(whichTime); Serial.println("s");
				}
			}
		}
	}
	memset(serialBuffer, 0, BUFFER_SIZE); //empty the buffer
  }
	//proto code for ArmMotor::update() function?
	switch(whichMotor){ // tells the appropriate motor to move with ArmMotor.budge()
		case MOTOR1:
			motor1.budge(whichDir);
			break;
		case '2':
			motor2.budge(whichDir);
			break;
		case '3':
			motor3.budge(whichDir);
			break;
		case '4':
			motor4.budge(whichDir);
			break;
		case '5':
			motor5.budge(whichDir);
			break;
		case '6':
			motor6.budge(whichDir);
			break;
	}
  
  /* //dcMotor.currentAngle = abs(dcMotor.encCount) / 5.555;
  if (!dcMotor.dcMovementDone) {
    //    Serial.print(dcMotor.encCount); Serial.print(","); Serial.println(dcMotor.currentAngle);
    analogWrite(DC_PWM_PIN, dcMotor.dir);
  }

   //  if (abs(dcMotor.currentAngle - dcMotor.desiredAngle) < 0.5 ) {
  //    analogWrite(DC_PWM_PIN, 127);
  //    dcMotor.dcMovementDone = true;
  //  }
*/
  /* //heartbeat : blink LED every 1 second
  int t2 = millis();
  if ( (t2 - startTime) > 1000) {
    startTime = t2;
    led_status = !led_status;
  }
  */
}

void parseCommand(char* line) {
  char* pch;
  int term = 0;
  int whichMotor = 0;

  /* Currently Interprets :

     S,StepperNumber,angle      --> Single stepper motor angle
     K,motor1angle, motor2angle --> Inverse kinematic
     R                          --> CW DC motor movement contatnt speed
     Q                          --> CCW DC motor movement constant speed
     X                          --> Stop the DC motor
     Z                          --> Home position on all steppers

    //TODO
     F,Servo angle              --> Servo finger movement
     D,dcMotor angle            --> Set dc motor angle constant speed
     E,stepper                  --> Enable or disable steppers to save power (no current draw)

  */

  pch = strtok(line, ",");  //start from the beginning of the line

/*
  if (*pch == 'F') {
    Serial.println("Mode: Servo movement");
    pch = strtok(NULL, ","); //skip the prefix "F"
    while (pch != NULL) {
      Serial.print("servo angle: ");
      Serial.println(pch);
      pch = strtok(NULL, ","); //continue from we left off
    }
  }
  else if (*pch == 'S') {
    Serial.println("Mode: Single Stepper");
    pch = strtok(NULL, ","); //skip the prefix "S"
    while (pch != NULL) {
      if (term == 0) { //first found term
        Serial.print("Motor: ");
        Serial.println(pch);
        whichMotor = atoi(pch);
      }
      if (term == 1) { //second found term
        Serial.print("Angle: ");
        Serial.println(pch);
        prepareMovement(whichMotor, atof(pch));
        startMovement();
      }
      term++;
      pch = strtok(NULL, ","); //continue from we left off
    }

  }
  else if (*pch == 'K') {
    Serial.println("Mode: Inverse Kinematics");
    pch = strtok(NULL, ","); //skip the prefix "K"
    while (pch != NULL) {
      if (term == 0) { //first found term
        Serial.print("motor 1 angle: ");
        Serial.println(pch);
        prepareMovement(0, atof(pch));
      }
      if (term == 1) { //second found term
        Serial.print("motor 2 angle: ");
        Serial.println(pch);
        prepareMovement(1, atof(pch));
      }
      if (term == 2) {
        Serial.print("motor 3 angle: ");
        Serial.println(pch);
      }
      term++;
      pch = strtok(NULL, ","); //continue from we left off
    }
    startMovement();
  }
  else if (*pch == 'Z') {
    Serial.println("Returning steppers to home position");
    prepareMovement(0, 1);
    prepareMovement(1, 1);
    startMovement();
  }
  else if (*pch == 'Q') {
    dcMotor.dcMovementDone = false;
    dcMotor.dir = 150;
  }
  else if (*pch == 'R') {
    dcMotor.dcMovementDone = false;
    dcMotor.dir = 100;
  }
  else if (*pch == 'X') {
    analogWrite(DC_PWM_PIN, 127);
    dcMotor.dcMovementDone = true;
  }
  else if (*pch == 'D') {
    Serial.println("Mode: DC motor");
    pch = strtok(NULL, ","); //skip the prefix "D"
    while (pch != NULL) {
      Serial.print("DC angle: ");
      Serial.println(pch);
      prepareDCMotor(atof(pch));
      pch = strtok(NULL, ","); //continue from we left off
    }
  }

  else {
    Serial.println("Command not found!");
  }

}
