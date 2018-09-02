#ifndef ARMMOTOR_H
#define ARMMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
//#include "AbtinEncoder.h"

// for 3.3v output
// pwm speed control
//#define DC_STOP 189 // motor is supposed to stop at 50% duty cycle (127/255)
// speed 0, slowest
#define DC_CW0 DC_STOP+15
#define DC_CCW0 DC_STOP-15
// speed 1
#define DC_CW1 DC_STOP+30
#define DC_CCW1 DC_STOP-30
// speed 2
#define DC_CW2 DC_STOP+50
#define DC_CCW2 DC_STOP-50
// speed 3, fastest
#define DC_CW3 255
#define DC_CCW3 DC_STOP-(255-DC_STOP)

// for 5v output
/*// pwm speed control
  //#define DC_STOP 127 // motor is supposed to stop at 50% duty cycle (127/255)
// speed 0, slowest
#define DC_CW0 DC_STOP+32
#define DC_CCW0 DC_STOP-32
// speed 1
#define DC_CW1 DC_STOP+64
#define DC_CCW1 DC_STOP-64
// speed 2
#define DC_CW2 DC_STOP+96
#define DC_CCW2 DC_STOP-96  
// speed 3, fastest
#define DC_CW3 255
#define DC_CCW3 1
*/

class DCMotor {
  public:
    //volatile int encoderCount;
    elapsedMillis sinceStart;

    //int gearRatio;
    //float maxCWAngle, maxCCWAngle;
    //float currentAngle, desiredAngle;

    //int maxSpeed;
    int cwSpeed, ccwSpeed;
    //bool movementDone;

    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    DCMotor(int pwmPin, int encA, int encB);
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

DCMotor::DCMotor(int pwmPin, int encA, int encB)://, void (*encoder_interrupt)(void)):
  pwmPin(pwmPin), encA(encA), encB(encB)
{
  /*encoder_interrupt = &encoder_interrupt;
    attachInterrupt(encA, encoder_interrupt, CHANGE);
    attachInterrupt(encB, encoder_interrupt, CHANGE);*/
}

void DCMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("turning dc clockwise");
      rightCount++; leftCount--;
    }
    if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("turning dc counter-clockwise");
      leftCount++; rightCount--;
    }
    Serial.print("right dc count ");Serial.println(rightCount);
    Serial.print("left dc count ");Serial.println(leftCount);
    Serial.print("right dc bool ");Serial.println(canTurnRight);
    Serial.print("left dc bool ");Serial.println(canTurnLeft);
    switch (budgeSpeed) {
      case 0:
        cwSpeed = DC_CW0; ccwSpeed = DC_CCW0;
        break;
      case 1:
        cwSpeed = DC_CW1; ccwSpeed = DC_CCW1;
        break;
      case 2:
        cwSpeed = DC_CW2; ccwSpeed = DC_CCW2;
        break;
      case 3:
        cwSpeed = DC_CW3; ccwSpeed = DC_CCW3;
        break;
    }
    Serial.print("setting dc speed level to ");Serial.println(budgeSpeed);
    sinceStart = 0;
    if (budgeDir == CLOCKWISE && canTurnRight) {
      analogWrite(pwmPin, cwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, DC_STOP); // sets duty cycle to 50% which corresponds to 0 speed
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      analogWrite(pwmPin, ccwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, DC_STOP); // sets duty cycle to 50% which corresponds to 0 speed
    }
    Serial.println("dc movement done");
    canTurnRight = false; canTurnLeft = false;
  }
}

#endif
