#include "PinSetup.h"
#include "DCMotor.h"
#include "ServoMotor.h"
#include "StepperMotor.h"

#define BAUD_RATE 115200 // serial baud rate
#define BUFFER_SIZE 100  // size of the buffer for the serial commands

int motor;
int whichMotor;
int whichDir;
int whichSpeed;
unsigned int whichTime;
char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
int tempSpeedVar;
unsigned int tempTimeVar;

DCMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B);

void setup() {
  pinSetup();
  Serial.begin(BAUD_RATE); Serial.setTimeout(50); // checks serial port every 50ms
}

void loop() {

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received

    char* msgElem = strtok(serialBuffer, " "); // look for first element (first tag)
    if (String(msgElem) == "motor") { // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok(NULL, " "); // go to next msg element (motor number)
      motor = atoi(msgElem);
      if (motor > 0 && motor < 10) whichMotor = motor;
      else Serial.println("bad motor number");
      Serial.print("parsed motor "); Serial.println(whichMotor);
      msgElem = strtok(NULL, " "); // find the next message element (direction tag)
      if (String(msgElem) == "direction") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok(NULL, " "); // go to next msg element (direction)
        switch (*msgElem) { // determines motor direction
          case '0': // arbitrarily (for now) decided 0 is clockwise
            whichDir = CLOCKWISE;
            Serial.println("parsed direction clockwise");
            break;
          case '1': // arbitrarily (for now) decided 1 is counter-clockwise
            whichDir = COUNTER_CLOCKWISE;
            Serial.println("parsed direction counter-clockwise");
            break;
        }
        msgElem = strtok(NULL, " "); // find the next message element (speed tag)
        if (String(msgElem) == "speed") { // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok(NULL, " "); // find the next message element (integer representing speed level)
          tempSpeedVar = atoi(msgElem); // converts to int
          if (tempSpeedVar <= MAX_SPEED) { // make sure the subtraction made sense. Tf it's above 9, it doesn't
            whichSpeed = tempSpeedVar; // set the actual speed
            Serial.print("parsed speed level: "); Serial.println(whichSpeed);
          }
        }
        msgElem = strtok(NULL, " "); // find the next message element (time tag)
        if (String(msgElem) == "time") { // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok(NULL, " "); // find the next message element (time in seconds)
          tempTimeVar = atoi(msgElem); // converts to int
          if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
            whichTime = tempTimeVar;
            Serial.print("parsed time interval ");Serial.print(whichTime); Serial.println("ms");
          }
        }
      }
    }
    memset(serialBuffer, 0, BUFFER_SIZE); //empty the buffer
  }

  if (whichMotor == MOTOR2) {
    Serial.print("motor "); Serial.print(whichMotor); Serial.println(" to move");
    motor2.budge(whichDir, whichSpeed, whichTime);
  }
  whichMotor = 0; // this was a quick fix so that at the next loop it will wait for a new message
  
}


