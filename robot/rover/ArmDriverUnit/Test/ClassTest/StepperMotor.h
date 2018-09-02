#ifndef ARMMOTOR_H
#define ARMMOTOR_H

#include <Arduino.h>
#include "AbtinEncoder.h"

// time interval between stepper steps
#define STEP_INTERVAL0 10
#define STEP_INTERVAL1 25
#define STEP_INTERVAL2 50
#define STEP_INTERVAL3 100

class StepperMotor {
  public:
    volatile int encoderCount;
    elapsedMillis sinceStart;
    
    int gearRatio;
    float maxCWAngle, maxCCWAngle;
    float currentAngle, desiredAngle;

    //int maxSpeed;
    int cwSpeed, ccwSpeed;
    bool movementDone;
    
    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    StepperMotor(int enablePin, int dirPin, int stepPin, int encA, int encB, void (*ISR)(void);
    void setMaxCWAngle(int angle);
    void setMaxCCWAngle(int angle);
    //void setMaxSpeed();
    void setDesiredAngle(float angle);
    float getCurrentAngle();
    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    void update();

  private:
    int enablePin, dirPin, stepPin;
    int encA, encB;

    void (*encoderInterrupt)(void);
};

StepperMotor::StepperMotor(int enablePin, int dirPin, int stepPin, int encA, int encB, void (*encoder_interrupt)(void)):
    enablePin(enablePin),dirPin(dirPin),stepPin(stepPin),encA(encA),encB(encB)
{
    encoder_interrupt = &encoder_interrupt;
    attachInterrupt(encA, encoder_interrupt, CHANGE);
    attachInterrupt(encB, encoder_interrupt, CHANGE);
}

StepperMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime){
	if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
		if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
		  canTurnRight = true;
		  rightCount++; leftCount--;
		}
		if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
		  canTurnLeft = true;
		  leftCount++; rightCount--;
		}
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
		if (budgeDir == CLOCKWISE && canTurnRight) {
		  digitalWriteFast(dirPin, HIGH);
		  digitalWriteFast(enablePin, HIGH);
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
		  digitalWriteFast(enablePin, HIGH);
		  sinceStart = 0;
		  while (sinceStart < budgeTime) {
			// motor driver is fast enough to recognize this quickly rising and falling edge
			digitalWriteFast(stepPin, HIGH);
			digitalWriteFast(stepPin, LOW);
			sinceStep = 0;
			while (sinceStep < stepInterval) ; // wait until it's time to step again
		  }
		}
	digitalWriteFast(enablePin, LOW); // be sure to disconnect power to stepper so it doesn't get hot / drain power
	}
}

#endif
