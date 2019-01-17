#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RoverMotor.h"

// time interval between stepper steps
#define STEP_INTERVAL0 35
#define STEP_INTERVAL1 25
#define STEP_INTERVAL2 10
#define STEP_INTERVAL3 3

class StepperMotor : public RoverMotor {
  public:
    static int numStepperMotors;

    StepperMotor(int enablePin, int dirPin, int stepPin, float gearRatio); // if no encoder
    StepperMotor(int enablePin, int dirPin, int stepPin, // if all data is known
                 int encA, int encB, uint32_t port, int shift, int encRes, float gearRatio);

    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED,
               unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    void setVelocity(int motorDir, int motorSpeed);
    float getCurrentAngle();

  private:
    int enablePin, directionPin, stepPin;
    elapsedMillis sinceStep;
    unsigned int stepInterval;
};

int StepperMotor::numStepperMotors = 0; // must initialize variable outside of class

StepperMotor::StepperMotor(int enablePin, int dirPin, int stepPin, float gearRatio): // if no encoder
  enablePin(enablePin), directionPin(dirPin), stepPin(stepPin)
{
  numStepperMotors++;
  this->gearRatio = gearRatio;
  hasEncoder = false;
}

StepperMotor::StepperMotor(int enablePin, int dirPin, int stepPin, // if all data is known
                           int encA, int encB, uint32_t port, int shift, int encRes, float gearRatio):
  enablePin(enablePin), directionPin(dirPin), stepPin(stepPin)
{
  numStepperMotors++;
  this->encoderPinA = encA;
  this->encoderPinB = encB;
  this->encoderPort = port;
  this->encoderShift = shift;
  this->encoderResolution = encRes;
  this->gearRatio = gearRatio;
  hasEncoder = true;
}

void StepperMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= COUNTER_CLOCKWISE && budgeSpeed <= MAX_SPEED
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
      movementDone = false;
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
      Serial.print("setting step interval to "); Serial.println(stepInterval); Serial.println("starting stepper movement");
      digitalWriteFast(directionPin, HIGH);
      digitalWriteFast(enablePin, LOW);
      sinceStart = 0;
      while (sinceStart < budgeTime) {
        // motor driver is fast enough to recognize this quickly rising and falling edge
        digitalWriteFast(stepPin, HIGH);
        digitalWriteFast(stepPin, LOW);
        sinceStep = 0;
        while (sinceStep < stepInterval) ; // wait until it's time to step again
      }
      movementDone = true; Serial.println("stepper movement done");
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      movementDone = false;
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
      Serial.print("setting step interval to "); Serial.println(stepInterval); Serial.println("starting stepper movement");
      digitalWriteFast(directionPin, LOW);
      digitalWriteFast(enablePin, LOW);
      sinceStart = 0;
      while (sinceStart < budgeTime) {
        // motor driver is fast enough to recognize this quickly rising and falling edge
        digitalWriteFast(stepPin, HIGH);
        digitalWriteFast(stepPin, LOW);
        sinceStep = 0;
        while (sinceStep < stepInterval) ; // wait until it's time to step again
      }
      movementDone = true; Serial.println("stepper movement done");
    }
    digitalWriteFast(enablePin, HIGH); // be sure to disconnect power to stepper so it doesn't get hot / drain power
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

float StepperMotor::getCurrentAngle() {
  if (hasEncoder) {
    currentAngle = (float)encoderCount * 360.0 / (encoderResolution * gearRatio);
    return currentAngle;
  }
  else {
    Serial.println("$E,Error: motor does not have encoder");
    return 404.0; // wants a return value, at least this value should be invalid
  }
}

#endif
