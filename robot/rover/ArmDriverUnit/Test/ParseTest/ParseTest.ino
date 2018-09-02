#include "ArmMotor.h"
#include "PinSetup.h"
#include "Parser.h"

#define BAUD_RATE 115200 // serial baud rate
#define BUFFER_SIZE 100  // size of the buffer for the serial commands

int whichMotor;
int whichDir;
int whichSpeed;
unsigned int whichTime;
char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
int tempSpeedVar;
unsigned int tempTimeVar;

Parser parser;

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received

    Parser.parseMotorCommand();
  }
  
/*
  if(whichMotor>0) {Serial.print("motor ");Serial.print(whichMotor);Serial.println(" to move");}
  
  //proto code for ArmMotor::update() function?
  switch (whichMotor) { // tells the appropriate motor to move with ArmMotor.budge()
    case MOTOR1:
      motor1.budge(whichDir, whichSpeed, whichTime);
      break;
    case MOTOR2:
      motor2.budge(whichDir, whichSpeed, whichTime);
      break;
    case MOTOR3:
      motor3.budge(whichDir, whichSpeed, whichTime);
      break;
    case MOTOR4:
      motor4.budge(whichDir, whichSpeed, whichTime);
      break;
    case MOTOR5:
      motor5.budge(whichDir, whichSpeed, whichTime);
      break;
    case MOTOR6:
      motor6.budge(whichDir, whichSpeed, whichTime);
      break;
  }
  */
  //whichMotor = 0; // this was a quick fix so that at the next loop it will wait for a new message

}