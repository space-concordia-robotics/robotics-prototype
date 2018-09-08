#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RoverMotor.h"

// for 3.3v output
// pwm speed control
//#define SERVO_STOP 189 // motor is supposed to stop at 50% duty cycle (127/255)
// speed 0, slowest
#define SERVO_CW0 SERVO_STOP+15
#define SERVO_CCW0 SERVO_STOP-15
// speed 1
#define SERVO_CW1 SERVO_STOP+30
#define SERVO_CCW1 SERVO_STOP-30
// speed 2
#define SERVO_CW2 SERVO_STOP+50
#define SERVO_CCW2 SERVO_STOP-50
// speed 3, fastest
#define SERVO_CW3 255
#define SERVO_CCW3 SERVO_STOP-(255-SERVO_STOP)

// for 5v output
/*// pwm speed control
  //#define SERVO_STOP 127 // motor is supposed to stop at 50% duty cycle (127/255)
  // speed 0, slowest
  #define SERVO_CW0 SERVO_STOP+32
  #define SERVO_CCW0 SERVO_STOP-32
  // speed 1
  #define SERVO_CW1 SERVO_STOP+64
  #define SERVO_CCW1 SERVO_STOP-64
  // speed 2
  #define SERVO_CW2 SERVO_STOP+96
  #define SERVO_CCW2 SERVO_STOP-96
  // speed 3, fastest
  #define SERVO_CW3 255
  #define SERVO_CCW3 1
*/

class ServoMotor : public RoverMotor {
  public:
    //volatile int encoderCount;

    static int numServoMotors;
    //int gearRatio;

    ServoMotor(int pwmPin);

    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    //float getCurrentAngle();

  private:
    int pwmPin;
};

int ServoMotor::numServoMotors = 0; // C++ is annoying and we need this to initialize the variable to 0

ServoMotor::ServoMotor(int pwmPin):
  pwmPin(pwmPin)
{
  numServoMotors++;
}

void ServoMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= COUNTER_CLOCKWISE && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("preparing to turn servo clockwise");
      rightCount++; leftCount--;
    }
    else if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("preparing to turn servo counter-clockwise");
      leftCount++; rightCount--;
    }
    else Serial.println("max turn count reached");
    Serial.print("right servo count "); Serial.println(rightCount);
    Serial.print("left servo count "); Serial.println(leftCount);

    if (budgeDir == CLOCKWISE && canTurnRight) {
      movementDone = false;
      switch (budgeSpeed) {
        case 0:
          cwSpeed = SERVO_CW0; ccwSpeed = SERVO_CCW0;
          break;
        case 1:
          cwSpeed = SERVO_CW1; ccwSpeed = SERVO_CCW1;
          break;
        case 2:
          cwSpeed = SERVO_CW2; ccwSpeed = SERVO_CCW2;
          break;
        case 3:
          cwSpeed = SERVO_CW3; ccwSpeed = SERVO_CCW3;
          break;
      }
      Serial.print("setting servo speed level to "); Serial.println(budgeSpeed); Serial.println("starting servo movement");
      sinceStart = 0;
      analogWrite(pwmPin, cwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, SERVO_STOP); // sets duty cycle to 50% which corresponds to 0 speed
      movementDone = true; Serial.println("servo movement done");
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      movementDone = false;
      switch (budgeSpeed) {
        case 0:
          cwSpeed = SERVO_CW0; ccwSpeed = SERVO_CCW0;
          break;
        case 1:
          cwSpeed = SERVO_CW1; ccwSpeed = SERVO_CCW1;
          break;
        case 2:
          cwSpeed = SERVO_CW2; ccwSpeed = SERVO_CCW2;
          break;
        case 3:
          cwSpeed = SERVO_CW3; ccwSpeed = SERVO_CCW3;
          break;
      }
      Serial.print("setting servo speed level to "); Serial.println(budgeSpeed);
      sinceStart = 0;
      analogWrite(pwmPin, ccwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, SERVO_STOP); // sets duty cycle to 50% which corresponds to 0 speed
      movementDone = true; Serial.println("servo movement done");
    }
    canTurnRight = false; canTurnLeft = false;
  }
}

#endif
