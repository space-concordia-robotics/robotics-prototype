#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <Arduino.h>
#include "PidController.h"
enum motor_direction
{
  CW = -1, CCW = 1
}; // defines motor directions, CW CLOCKWISE, CCW COUNTER_CLOCKWISE

enum loop_state
{
  OPEN_LOOP = 1, CLOSED_LOOP
}; // defines whether motor control is open loop or closed loop

class RobotMotor
{
  public:
  // these variables are set at start and normally don't change during the main loop
  static int numMotors; // keeps track of how many motors there are
  int encoderPinA, encoderPinB;
  float gearRatio, gearRatioReciprocal; // calculating this beforehand improves speed of floating point calculations
  float encoderResolutionReciprocal; // calculating this beforehand improves speed of floating point calculations
  bool isOpenLoop; // decides whether to use the PID or not
  volatile int rotationDirection;
  // int maxSpeed;
  PidController pidController; // used for speed and angle control
  // these variables change during the main loop
  volatile long encoderCount; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
  volatile bool movementDone; // this variable is what allows the timer interrupts to make motors turn. can be updated within said interrupts
  // setup functions
  RobotMotor();
  void attachEncoder(int encA, int encB);
  // void attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes);
  bool hasEncoder;
  virtual void setVelocity(int motorDir, float motorSpeed) = 0; // sets motor speed and direction until next timer interrupt
  virtual void stopRotation(void) = 0;
  // void setMaxSpeed();
  int calcDirection(float error); // updates rotationDirection based on the angular error inputted
  bool setDesiredVelocity(float velocity); // if the angle is valid, update desiredAngle and return true. else return false.
  float getDesiredVelocity(void); // return copy of the desired angle, not a reference to it
  bool calcCurrentVelocity(void);
  float getCurrentVelocity(void);
  void switchDirectionLogic(void); // tells the motor to reverse the direction for a motor's control... does this need to be virtual?
  int getDirectionLogic(void); // returns the directionModifier;
  private:
  // doesn't really make sense to have any private variables for this parent class.
  // note that virtual functions must be public in order for them to be accessible from motorArray[]
  protected:
  // the following variables are specific to encoders
  uint32_t encoderPort; // address of the port connected to a particular encoder pin
  int encoderShift; // how many bits to shift over to find the encoder pin state
  int encoderResolution; // ticks per revolution
  volatile float currentAngle; // can be updated within timer interrupts
  float desiredAngle;
  int directionModifier;
};



void RobotMotor::attachEncoder(int encA, int encB) // :
// encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{

  hasEncoder = true;
  encoderPinA = encA;
  encoderPinB = encB;
  // encoderPort = port;
  // encoderShift = shift;
  // encoderResolution = encRes;
  // encoderResolutionReciprocal = 1 / (float) encRes;
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
}


int RobotMotor::calcDirection(float error)
{
  if (error >= 0)
  {
    rotationDirection = directionModifier * CCW;
  }
  else
  {
    rotationDirection = directionModifier * CW;
  }
  return rotationDirection;
}
void RobotMotor::setDesiredVelocity(float velocity)
{
    desiredVelocity = velocity;
}

float RobotMotor::getDesiredVelocity(void)
{
  return desiredVelocity;
}

int RobotMotor::calcDirection(float error)
{
  if (error >= 0)
  {
    rotationDirection = directionModifier * CCW;
  }
  else
  {
    rotationDirection = directionModifier * CW;
  }
  return rotationDirection;
}
void RobotMotor::switchDirectionLogic(void)
{
  directionModifier = directionModifier * -1;
}

int RobotMotor::getDirectionLogic(void)
{
  return directionModifier;
}

bool RobotMotor::calcCurrentVelocity(void)
{
  if (hasEncoder)
  {
    currentVelocity = (float) encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
    return true;
  }
  else
  {
    return false;
  }
}

float RobotMotor::getCurrentVelocity(void)
{
  return currentVelocity;
}

void RobotMotor::setCurrentVelocity(float velocity)
{
  currentVelocity = velocity;
}

class DcMotor:public RobotMotor
{
  public:
    static int numDcMotors;
    // DcMotor(int pwmPin, int encA, int encB); // for sabertooth
    // for cytron
    DcMotor(int dirPin, int pwmPin, float gearRatio, float maxV);
    void stopRotation(void);
    void setVelocity(int motorDir, float motorSpeed); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    // stuff for open loop control
    unsigned int numMillis; // how many milliseconds for dc motor to reach desired position
    elapsedMillis timeCount; // how long has the dc motor been turning for
  private:
    int directionPin; // for new driver
    int pwmPin;
    float maxVelocity;
};

int DcMotor::numDcMotors = 0; // must initialize variable outside of class
// for sabertooth
// DcMotor::DcMotor(int pwmPin, int encA, int encB):
// pwmPin(pwmPin), encA(encA), encB(encB)
// for new driver
DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio, float maxV):// if no encoder
directionPin(dirPin), pwmPin(pwmPin), maxVelocity(maxV)
{
  numDcMotors++;
  // variables declared in RobotMotor require the this-> operator
  this -> gearRatio = gearRatio;
  this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  // hasEncoder = false;
  // openLoopSpeed = 0; // no speed by default;
  // openLoopGain = 1.0; // temp open loop control
  pinMode(directionPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

}

void DcMotor::stopRotation(void)
{
  analogWrite(pwmPin, 0); // 0 for cytron, different implementation for sabertooth
  movementDone = true;
}

void DcMotor::setVelocity(int motorDir, float motorSpeed)
{

    motorSpeed = fabs(motorSpeed);
  // makes sure the speed is within the limits set in the pid during setup
  if (motorSpeed * motorDir > pidController.getMaxOutputValue())
  {
    motorSpeed = pidController.getMaxOutputValue();
  }
  if (motorSpeed * motorDir < pidController.getMinOutputValue())
  {
    motorSpeed = pidController.getMinOutputValue();
  }

  switch (motorDir)
  {
    case CW:
      digitalWrite(directionPin, LOW);
      break;
    case CCW:
      digitalWrite(directionPin, HIGH);
      break;
  }
  int dutyCycle = motorSpeed * 255 / 100;
  analogWrite(pwmPin, dutyCycle);
}



#endif
