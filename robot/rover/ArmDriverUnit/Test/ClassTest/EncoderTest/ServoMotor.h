#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"

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

class ServoMotor {
  public:
    //volatile int encoderCount;
    elapsedMillis sinceStart;

    static int numServoMotors;
    //int gearRatio;
    //float maxCWAngle, maxCCWAngle;
    //float currentAngle, desiredAngle;

    //int maxSpeed;
    int cwSpeed, ccwSpeed;
    bool movementDone;

    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    ServoMotor(int pwmPin, int encA, int encB);
    //void setMaxCWAngle(int angle);
    //void setMaxCCWAngle(int angle);
    //void setMaxSpeed();
    //void setDesiredAngle(float angle);
    //float getCurrentAngle();
    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    //void update();

  private:
    int pwmPin;
    int encA, encB;

    //void (*encoderInterrupt)(void);
};

int ServoMotor::numServoMotors = 0; // C++ is annoying and we need this to initialize the variable to 0

ServoMotor::ServoMotor(int pwmPin, int encA, int encB)://, void (*encoder_interrupt)(void)):
  pwmPin(pwmPin), encA(encA), encB(encB)
{
  numServoMotors++;
  /*encoder_interrupt = &encoder_interrupt;
    attachInterrupt(encA, encoder_interrupt, CHANGE);
    attachInterrupt(encB, encoder_interrupt, CHANGE);*/
}

void ServoMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    movementDone = false;
    // following if statements ensure motor only moves if within count limit, updates current count
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("turning servo clockwise");
      rightCount++; leftCount--;
    }
    if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("turning servo counter-clockwise");
      leftCount++; rightCount--;
    }
    Serial.print("right servo count "); Serial.println(rightCount);
    Serial.print("left servo count "); Serial.println(leftCount);
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
    if (budgeDir == CLOCKWISE && canTurnRight) {
      analogWrite(pwmPin, cwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, SERVO_STOP); // sets duty cycle to 50% which corresponds to 0 speed
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      analogWrite(pwmPin, ccwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, SERVO_STOP); // sets duty cycle to 50% which corresponds to 0 speed
    }
    movementDone = true; Serial.println("servo movement done");
    canTurnRight = false; canTurnLeft = false;
  }
}

#endif
