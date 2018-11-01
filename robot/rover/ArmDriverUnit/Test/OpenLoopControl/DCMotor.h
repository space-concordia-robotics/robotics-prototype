#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotMotor.h"

// for cytron
#define DC_S1 60
#define DC_S2 130
#define DC_S3 190
#define DC_S4 255

const int dcSpeedArray[] = {DC_S1, DC_S2, DC_S3, DC_S4};

// for 3.3v output, sabertooth
/*// pwm speed control
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
*/

// for 5v output, sabertooth
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

class DcMotor : public RobotMotor {

  public:
    static int numDcMotors;

    //DcMotor(int pwmPin, int encA, int encB); // for sabertooth

    // for new driver
    DcMotor(int dirPin, int pwmPin, float gearRatio);

    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED,
               unsigned int budgeTime = DEFAULT_BUDGE_TIME); // can go into ArmMotor

    bool calcTurningDuration(float angle); // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    
    void stopRotation(void);
    void setVelocity(int motorDir, int motorSpeed);
    float calcCurrentAngle(void);

    // stuff for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    float openLoopGain; // speed correction factor
    unsigned int numMillis; // how many milliseconds for dc motor to reach desired position
    elapsedMillis timeCount; // how long has the dc motor been turning for

  private:
    int directionPin; // for new driver
    int pwmPin;

};

int DcMotor::numDcMotors = 0; // must initialize variable outside of class

// for sabertooth
//DcMotor::DcMotor(int pwmPin, int encA, int encB):
//  pwmPin(pwmPin), encA(encA), encB(encB)

// for new driver
DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio): // if no encoder
  directionPin(dirPin), pwmPin(pwmPin)
{
  numDcMotors++;
  this->gearRatio = gearRatio;
  this->gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  hasEncoder = false;
  budgeMovementDone = true;

  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
}

/*
  void DcMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
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
    Serial.print("right dc count "); Serial.println(rightCount);
    Serial.print("left dc count "); Serial.println(leftCount);
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
    Serial.print("setting dc speed level to "); Serial.println(budgeSpeed);
    sinceStart = 0;
    if (budgeDir == CLOCKWISE && canTurnRight) {
      movementDone = false;
      analogWrite(pwmPin, cwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, DC_STOP); // sets duty cycle to 50% which corresponds to 0 speed
      movementDone = true; Serial.println("dc movement done");
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      movementDone = false;
      analogWrite(pwmPin, ccwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, DC_STOP); // sets duty cycle to 50% which corresponds to 0 speed
      movementDone = true; Serial.println("dc movement done");
    }
    canTurnRight = false; canTurnLeft = false;
  }
  }
*/

void DcMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= COUNTER_CLOCKWISE && budgeSpeed > 0 && budgeSpeed <= MAX_SPEED
      && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("preparing to turn dc clockwise");
      rightCount++; leftCount--;
    }
    else if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("preparing to turn dc counter-clockwise");
      leftCount++; rightCount--;
    }
    else Serial.println("max turn count reached");
    Serial.print("right dc count "); Serial.println(rightCount);
    Serial.print("left dc count "); Serial.println(leftCount);

    if (budgeDir == CLOCKWISE && canTurnRight) {
      budgeMovementDone = false;
      cwSpeed = ccwSpeed = dcSpeedArray[budgeSpeed];
      Serial.print("setting dc speed level to "); Serial.println(budgeSpeed); Serial.println("starting dc movement");
      digitalWrite(directionPin, LOW);
      sinceStart = 0;
      analogWrite(pwmPin, cwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, 0); // sets duty cycle to 50% which corresponds to 0 speed
      budgeMovementDone = true; Serial.println("dc movement done");
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      budgeMovementDone = false;
      cwSpeed = ccwSpeed = dcSpeedArray[budgeSpeed];
      Serial.print("setting dc speed level to "); Serial.println(budgeSpeed);
      sinceStart = 0;
      digitalWrite(directionPin, HIGH);
      analogWrite(pwmPin, ccwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, 0); // sets duty cycle to 50% which corresponds to 0 speed
      budgeMovementDone = true; Serial.println("dc movement done");
    }
    canTurnRight = false; canTurnLeft = false;
  }
}

/*
  void DcMotor::encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.println(encoderCount);
  oldEncoderState <<= 2; // move by two bits (previous state in top 2 bits)
  oldEncoderState |= ((encoderPort >> encoderShift) & 0x03);
  //     encoderPort corresponds to the state of all the pins on the port this encoder is connected to.
  //     shift it right by the amount previously determined based on the encoder pin and the corresponding internal GPIO bit
  //     now the current state is in the lowest 2 bits, so you clear the higher bits by doing a logical AND with 0x03 (0b00000011)
  //     you then logical OR this with the previous state's shifted form to obtain (prevstate 1 prevstate 2 currstate 1 currstate 2)
  //     the catch which is accounted for below is that oldEncoderState keeps getting right-shifted so you need to clear the higher bits after this operation too
  encoderCount += dir[(oldEncoderState & 0x0F)]; // clear the higher bits. The dir[] array corresponds to the correct direction for a specific set of prev and current encoder states
  }
*/

void DcMotor::stopRotation(void) {
  analogWrite(pwmPin, 0);
}

void DcMotor::setVelocity(int motorDir, int motorSpeed) {
  int dutyCycle;
  if (motorSpeed > pidController.maxOutputValue) motorSpeed = pidController.maxOutputValue;
  if (motorSpeed < pidController.minOutputValue) motorSpeed = pidController.minOutputValue;
  switch (motorDir) {
    case CLOCKWISE:
      digitalWrite(directionPin, LOW);
      break;
    case COUNTER_CLOCKWISE:
      digitalWrite(directionPin, HIGH);
      break;
  }
  dutyCycle = motorSpeed * 255 / 100;
  analogWrite(pwmPin, dutyCycle);
}

float DcMotor::calcCurrentAngle(void) {
  if (hasEncoder) {
    currentAngle = encoderCount * 360.0 * gearRatioReciprocal * (1 / encoderResolution);
    return currentAngle;
  }
  else {
    Serial.println("$E,Error: motor does not have encoder");
    return 40404040404.0; // wants a return value, at least this value should be invalid
  }
}

bool DcMotor::calcTurningDuration(float angle) {
  // if the error is big enough to justify movement
  // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
  if ( fabs(angle) > pidController.angleTolerance * gearRatioReciprocal) {
    numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
    //Serial.println(numMillis);
    return true;
  }
  else {
    return false;
  }
}

#endif
