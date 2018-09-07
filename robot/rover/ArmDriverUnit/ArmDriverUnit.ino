#include "PinSetup.h"
#include "ArmMotor.h"
//#include "SerialParser.h"

#define BAUD_RATE 115200 // serial baud rate
#define BUFFER_SIZE 100  // size of the buffer for the serial commands

    int whichMotor;
    int whichDir;
    int whichSpeed;
    unsigned int whichTime;
    char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
    int tempSpeedVar;
    unsigned int tempTimeVar;

//SerialParser serialParser;

// these objects were created outside of the setup function because the update() proto-code isn't in the right place yet
ArmMotor motor1(MOTOR1); // base stepper (rotation)
ArmMotor motor2(MOTOR2); // shoulder dc (flexion)
ArmMotor motor3(MOTOR3); // elbow stepper (flexion)
ArmMotor motor4(MOTOR4); // wrist stepper (flexion)
ArmMotor motor5(MOTOR5); // wrist servo (rotation)
ArmMotor motor6(MOTOR6); // end effector servo (pinching)

void setup() {
  pinSetup();
}

void loop() {

  //digitalWrite(LED_PIN, led_status);

  /*
  serialParser.parseSerial();

  int whichMotor = serialParser.whichMotor;
  int whichDir = serialParser.whichDir;
  int whichSpeed = serialParser.whichSpeed;
  unsigned int whichTime = serialParser.whichTime;
  */

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received

    //parseCommand(serialBuffer);           //parse the string

    /*
       instead of using the parse function i dumped the code here for now.
       This code figures out which motor and which direction, code further
       down actually tells the motors to move
    */

    //strtok_r() splits message by a delimiter string
    char* msgElem = strtok(serialBuffer, " "); // look for first element (first tag)
    if (String(msgElem) == "motor") { // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok(NULL, " "); // go to next msg element (motor number)
      switch (*msgElem) { // determines which motor being used
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
      Serial.print("motor "); Serial.println(whichMotor);
      msgElem = strtok(NULL, " "); // find the next message element (direction tag)
      if (String(msgElem) == "direction") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok(NULL, " "); // go to next msg element (direction)
        switch (*msgElem) { // determines motor direction
          case '0': // arbitrarily (for now) decided 0 is clockwise
            whichDir = CLOCKWISE;
            Serial.println("clockwise");
            break;
          case '1': // arbitrarily (for now) decided 1 is counter-clockwise
            whichDir = COUNTER_CLOCKWISE;
            Serial.println("counter-clockwise");
            break;
        }
        msgElem = strtok(NULL, " "); // find the next message element (speed tag)
        if (String(msgElem) == "speed") { // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok(NULL, " "); // find the next message element (integer representing speed level)
          tempSpeedVar = atoi(msgElem); // converts to int
          if (tempSpeedVar <= MAX_SPEED) { // make sure the subtraction made sense. Tf it's above 9, it doesn't
            whichSpeed = tempSpeedVar; // set the actual speed
            Serial.print("speed level: "); Serial.println(whichSpeed);
          }
        }
        msgElem = strtok(NULL, " "); // find the next message element (time tag)
        if (String(msgElem) == "time") { // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok(NULL, " "); // find the next message element (time in seconds)
          tempTimeVar = atoi(msgElem); // converts to int
          if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
            whichTime = tempTimeVar;
            Serial.print(whichTime); Serial.println("ms movement");
          }
        }
      }
    }
    memset(serialBuffer, 0, BUFFER_SIZE); //empty the buffer
  }

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
  whichMotor = 0; // this was a quick fix so that at the next loop it will wait for a new message

}


