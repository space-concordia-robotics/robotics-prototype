#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"

// time interval between stepper steps
#define STEP_INTERVAL0 35
#define STEP_INTERVAL1 25
#define STEP_INTERVAL2 10
#define STEP_INTERVAL3 3

class StepperMotor {
  public:
    volatile int encoderCount;
    elapsedMillis sinceStart;

    static int numStepperMotors;
    //int gearRatio;
    //float maxCWAngle, maxCCWAngle;
    //float currentAngle, desiredAngle;

    //int maxSpeed;

    int cwSpeed, ccwSpeed;
    bool movementDone;

    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    StepperMotor(int enablePin, int dirPin, int stepPin, int encA, int encB);//, void (*ISR)(void);
    //void setMaxCWAngle(int angle);
    //void setMaxCCWAngle(int angle);
    //void setMaxSpeed();
    //void setDesiredAngle(float angle);
    //float getCurrentAngle();
    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    //void update();

  private:
    int enablePin, dirPin, stepPin;
    elapsedMillis sinceStep;
    unsigned int stepInterval;
    int encA, encB;

    //void (*encoderInterrupt)(void);
};

int StepperMotor::numStepperMotors = 0; // C++ is annoying and we need this to initialize the variable to 0

StepperMotor::StepperMotor(int enablePin, int dirPin, int stepPin, int encA, int encB)://, void (*encoder_interrupt)(void)):
  enablePin(enablePin), dirPin(dirPin), stepPin(stepPin), encA(encA), encB(encB)
{
  numStepperMotors++;
  /*encoder_interrupt = &encoder_interrupt;
    attachInterrupt(encA, encoder_interrupt, CHANGE);
    attachInterrupt(encB, encoder_interrupt, CHANGE);*/
}

void StepperMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
    movementDone = false;
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("turning stepper clockwise");
      rightCount++; leftCount--;
    }
    if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("turning stepper counter-clockwise");
      leftCount++; rightCount--;
    }
    Serial.print("right stepper count "); Serial.println(rightCount);
    Serial.print("left stepper count "); Serial.println(leftCount);
    switch (budgeSpeed) {
      case 0:
        stepInterval = STEP_INTERVAL0;
        break;
      case 1:
        stepInterval = STEP_INTERVAL1;
        break;
      case 2:
        stepInterval = STEP_INTERVAL2;
        break;
      case 3:
        stepInterval = STEP_INTERVAL3;
        break;
    }
    Serial.print("setting step interval to "); Serial.println(stepInterval);
    if (budgeDir == CLOCKWISE && canTurnRight) {
      digitalWriteFast(dirPin, HIGH);
      digitalWriteFast(enablePin, LOW);
      sinceStart = 0;
      while (sinceStart < budgeTime) {
        // motor driver is fast enough to recognize this quickly rising and falling edge
        digitalWriteFast(stepPin, HIGH);
        digitalWriteFast(stepPin, LOW);
        sinceStep = 0;
        while (sinceStep < stepInterval) ; // wait until it's time to step again
      }
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      digitalWriteFast(dirPin, LOW);
      digitalWriteFast(enablePin, LOW);
      sinceStart = 0;
      while (sinceStart < budgeTime) {
        // motor driver is fast enough to recognize this quickly rising and falling edge
        digitalWriteFast(stepPin, HIGH);
        digitalWriteFast(stepPin, LOW);
        sinceStep = 0;
        while (sinceStep < stepInterval) ; // wait until it's time to step again
      }
    }
    movementDone = true; Serial.println("stepper movement done");
    digitalWriteFast(enablePin, HIGH); // be sure to disconnect power to stepper so it doesn't get hot / drain power
    canTurnRight = false; canTurnLeft = false;
  }
}

#endif
