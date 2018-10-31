#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotMotor.h"

// time interval between stepper steps
#define STEP_INTERVAL1 35
#define STEP_INTERVAL2 25
#define STEP_INTERVAL3 10
#define STEP_INTERVAL4 3

#define FULL_STEP 1
#define HALF_STEP 0.5
#define QUARTER_STEP 0.25
#define EIGHTH_STEP 0.125
#define SIXTEENTH_STEP 0.0625

const int stepIntervalArray[] = {STEP_INTERVAL1, STEP_INTERVAL2, STEP_INTERVAL3, STEP_INTERVAL4};

class StepperMotor : public RobotMotor {
  public:
    static int numStepperMotors;
    float stepResolution;

    StepperMotor(int enablePin, int dirPin, int stepPin, float stepRes, float stepMode, float gearRatio);
    void singleStep(int dir);
    void enablePower(void);
    void disablePower(void);

    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED,
               unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    void setVelocity(int motorDir, int motorSpeed);
    float calcCurrentAngle(void);

    // stuff for open loop control
    int openLoopDir; // public variable for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    float openLoopGain; // speed correction factor
    int numSteps; // how many steps to take for stepper to reach desired position
    volatile int stepCount; // how many steps the stepper has taken since it started moving

  private:
    int enablePin, directionPin, stepPin;
    elapsedMillis sinceStep;
    unsigned int stepInterval;
    float fullStepResolution, steppingMode;
};

int StepperMotor::numStepperMotors = 0; // must initialize variable outside of class

StepperMotor::StepperMotor(int enablePin, int dirPin, int stepPin, float stepRes, float stepMode, float gearRatio): // if no encoder
  enablePin(enablePin), directionPin(dirPin), stepPin(stepPin), fullStepResolution(stepRes), steppingMode(stepMode)
{
  numStepperMotors++;
  this->gearRatio = gearRatio;
  this->gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  hasEncoder = false;
  stepResolution = fullStepResolution * steppingMode;
  budgeMovementDone = true;

  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
}

void StepperMotor::enablePower(void) {
  digitalWriteFast(enablePin, LOW);
}

void StepperMotor::disablePower(void) {
  digitalWriteFast(enablePin, HIGH);
}

void StepperMotor::singleStep(int dir) {
  switch (dir) {
    case CLOCKWISE:
      digitalWriteFast(directionPin, HIGH);
      break;
    case COUNTER_CLOCKWISE:
      digitalWriteFast(directionPin, LOW);
      break;
  }
  digitalWriteFast(stepPin, HIGH);
  digitalWriteFast(stepPin, LOW);
}

void StepperMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if ( (budgeDir == COUNTER_CLOCKWISE || budgeDir == CLOCKWISE) && budgeSpeed > 0 && budgeSpeed <= MAX_SPEED
       && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("preparing to turn stepper clockwise");
      rightCount++; leftCount--;
    }
    else if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("preparing to turn stepper counter-clockwise");
      leftCount++; rightCount--;
    }
    else Serial.println("max turn count reached");
    Serial.print("right stepper count "); Serial.println(rightCount);
    Serial.print("left stepper count "); Serial.println(leftCount);

    if (budgeDir == CLOCKWISE && canTurnRight) {
      budgeMovementDone = false;
      stepInterval = stepIntervalArray[budgeSpeed - 1];
      Serial.print("setting step interval to "); Serial.println(stepInterval); Serial.println("starting stepper movement");
      digitalWriteFast(directionPin, HIGH);
      enablePower();
      sinceStart = 0;
      while (sinceStart < budgeTime) {
        singleStep(budgeDir);
        sinceStep = 0;
        while (sinceStep < stepInterval) ; // wait until it's time to step again
      }
      budgeMovementDone = true; Serial.println("stepper movement done");
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      budgeMovementDone = false;
      stepInterval = stepIntervalArray[budgeSpeed - 1];
      Serial.print("setting step interval to "); Serial.println(stepInterval); Serial.println("starting stepper movement");
      digitalWriteFast(directionPin, LOW);
      enablePower();
      sinceStart = 0;
      while (sinceStart < budgeTime) {
        singleStep(budgeDir);
        sinceStep = 0;
        while (sinceStep < stepInterval) ; // wait until it's time to step again
      }
      budgeMovementDone = true; Serial.println("stepper movement done");
    }
    disablePower(); // be sure to disconnect power to stepper so it doesn't get hot / drain power
    canTurnRight = false; canTurnLeft = false;
  }
}

/*
  void StepperMotor::encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0; //0b1011; // solves bug where the encoder counts backwards for one count... but this may not work in practise
  Serial.println(encoderCount);
  oldEncoderState <<= 2; // move by two bits (previous state in top 2 bits)
  oldEncoderState |= ((encoderPort >> encoderShift) & 0x03);
  Serial.println(encoderShift);
  Serial.println(encoderPort, BIN);
  Serial.println(oldEncoderState, BIN);
  //     encoderPort corresponds to the state of all the pins on the port this encoder is connected to.
  //     shift it right by the amount previously determined based on the encoder pin and the corresponding internal GPIO bit
  //     now the current state is in the lowest 2 bits, so you clear the higher bits by doing a logical AND with 0x03 (0b00000011)
  //     you then logical OR this with the previous state's shifted form to obtain (prevstate 1 prevstate 2 currstate 1 currstate 2)
  //     the catch which is accounted for below is that oldEncoderState keeps getting right-shifted so you need to clear the higher bits after this operation too
  encoderCount += dir[(oldEncoderState & 0x0F)]; // clear the higher bits. The dir[] array corresponds to the correct direction for a specific set of prev and current encoder states
  }
*/

void StepperMotor::setVelocity(int motorDir, int motorSpeed) {
  ;
}

float StepperMotor::calcCurrentAngle(void) {
  if (hasEncoder) {
    currentAngle = encoderCount * 360.0 * gearRatioReciprocal * (1 / encoderResolution);
    return currentAngle;
  }
  else {
    Serial.println("$E,Error: motor does not have encoder");
    return 40404040404.0; // wants a return value, at least this value should be invalid
  }
}

#endif
