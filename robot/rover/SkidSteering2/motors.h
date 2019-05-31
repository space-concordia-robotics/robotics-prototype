#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <Arduino.h>
#include "PidController.h"

/* wheel motors */
#define PULSES_PER_REV     14
#define GEAR_RATIO         188.61
#define MAX_RPM            30

// defines motor directions, CW CLOCKWISE, CCW COUNTER_CLOCKWISE
enum motor_direction { CW = -1, CCW = 1 };
// defines whether motor control is open loop or closed loop
enum loop_state { OPEN_LOOP = 1, CLOSED_LOOP };

class RobotMotor {
  public:
    // these variables are set at start and normally don't change during the main loop
    static int numMotors; // keeps track of how many motors there are
    int encoderPinA, encoderPinB;
    int output_pwm;
    int prev_output_pwm;
    float acc;
    bool accLimit = true;
    String motorName;
    volatile unsigned long  dt;
    volatile unsigned long  dt2;
    volatile unsigned long prevTime;
    volatile unsigned long prevTime2;
    float gearRatio, gearRatioReciprocal; // calculating this beforehand improves speed of floating point calculations
    float encoderResolutionReciprocal; // calculating this beforehand improves speed of floating point calculations
    bool isOpenLoop = true; // decides whether to use the PID or not
    volatile int rotationDirection = CCW;
    String direction;
    // int maxSpeed;
    PidController pidController; // used for speed and angle control
    // these variables change during the main loop
    volatile long encoderCount = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile long aveCount = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile long prevCount = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile long prevVelocity = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile bool movementDone; // this variable is what allows the timer interrupts to make motors turn. can be updated within said interrupts
    // setup functions
    RobotMotor();
    void attachEncoder(int encA, int encB, int encRes);
    // void attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes);
    bool hasEncoder;

    // void setMaxSpeed();
    int calcDirection(float error); // updates rotationDirection based on the angular error inputted
    int getDirection(void); // updates rotationDirection based on the angular error inputted
    void setDesiredVelocity(float velocity); // if the angle is valid, update desiredAngle and return true. else return false.
    void setDesiredDirection(float direction); // Assign a direction.
    float getDesiredVelocity(void); // return copy of the desired angle, not a reference to it
    void calcCurrentVelocity();
    float getCurrentVelocity(void);
    void setCurrentVelocity(float velocity);
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
    volatile float currentVelocity; // can be updated within timer interrupts
    float desiredVelocity;
    int directionModifier;
    int desiredDirection;
};

int RobotMotor::numMotors = 0;

RobotMotor::RobotMotor() {
  numMotors++;
}

void RobotMotor::attachEncoder(int encA, int encB, int encRes) // :
// encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{
  hasEncoder = true;
  encoderPinA = encA;
  encoderPinB = encB;
  // encoderPort = port;
  // encoderShift = shift;
  encoderResolution = encRes;
  encoderResolutionReciprocal = 1 / (float) encRes;
}

int RobotMotor::calcDirection(float error) {
  if (error >= 0) {
    rotationDirection = directionModifier * CCW;
  }
  else {
    rotationDirection = directionModifier * CW;
  }
  return rotationDirection;
}
void RobotMotor::setDesiredDirection(float direction) {
  rotationDirection = direction;
}

int RobotMotor::getDirection(void) {
  return rotationDirection;
}
void RobotMotor::setDesiredVelocity(float velocity) {
  desiredVelocity = velocity;
}

float RobotMotor::getDesiredVelocity(void) {
  return desiredVelocity;
}

void RobotMotor::switchDirectionLogic(void) {
  directionModifier = directionModifier * -1;
}

int RobotMotor::getDirectionLogic(void) {
  return directionModifier;
}

void RobotMotor::calcCurrentVelocity() {
  if (dt <= 0 || encoderCount <= 0) {
    currentVelocity = 0;
  }
  else {
    currentVelocity = (float) (encoderCount * 60000000.0 * gearRatioReciprocal * encoderResolutionReciprocal / (float) (dt));
  }
  //    Serial.println(currentVelocity);
  //    Serial.println(motorName + " " + String(currentVelocity));
  encoderCount = 0;
  dt = 0 ;
  //  Serial.println(dt/1000);
}

float RobotMotor::getCurrentVelocity(void) {
  return currentVelocity;
}

void RobotMotor::setCurrentVelocity(float velocity) {
  currentVelocity = velocity;
}

class DcMotor: public RobotMotor {
  public:
    static int numDcMotors;
    DcMotor(int dirPin, int pwmPin, float gearRatio, String motorName);
    void stopRotation(void);
    int getPwmPin(void);
    void setVelocity(int motorDir, float desiredVelocity, volatile float currentVelocity); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    void setVelocityNoPID(int motorDir, float desiredVelocity); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    // stuff for open loop control
    int directionPin; // for new driver

    unsigned int numMillis; // how many milliseconds for dc motor to reach desired position
    elapsedMillis timeCount; // how long has the dc motor been turning for
  private:
    int pwmPin;
    float maxVelocity;
};

int DcMotor::numDcMotors = 0; // must initialize variable outside of class
// for sabertooth
// DcMotor::DcMotor(int pwmPin, int encA, int encB):
// pwmPin(pwmPin), encA(encA), encB(encB)
// for new driver
DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio, String motorName):// if no encoder
  directionPin(dirPin), pwmPin(pwmPin)
{
  numDcMotors++;
  // variables declared in RobotMotor require the this-> operator
  this -> gearRatio = gearRatio;
  this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  this -> motorName = motorName;
  // hasEncoder = false;
  // openLoopSpeed = 0; // no speed by default;
  // openLoopGain = 1.0; // temp open loop control
}

int DcMotor::getPwmPin(void) {
  return pwmPin;
}
void DcMotor::stopRotation(void) {
  analogWrite(pwmPin, 0); // 0 for cytron, different implementation for sabertooth
  movementDone = true;
}

void DcMotor::setVelocity(int motorDir, float desiredVelocity, volatile float currentVelocity) {
  if (isOpenLoop) {
    switch (motorDir) {
      case CW:
        digitalWrite(directionPin, LOW);
        this -> direction = "CW";
        break;
      case CCW:
        digitalWrite(directionPin, HIGH);
        this -> direction = "CCW";

        break;
    }
    output_pwm = desiredVelocity;

  }
  else if (!isOpenLoop) {
    // makes sure the speed is within the limits set in the pid during setup
    if (desiredVelocity > 30) {
      desiredVelocity = 30;
    } else if (desiredVelocity < 0) {
      desiredVelocity = 0;
    }
    output_pwm = pidController.updatePID(currentVelocity, desiredVelocity);

  }
  this -> desiredVelocity = output_pwm;

  /* Acceleration limiter */
  if (accLimit) {
    dt2 = micros() - prevTime2;
    acc = ((float) output_pwm - (float) prev_output_pwm) / (float) dt2;

    if (abs(acc) > 0.00051) {  // 0.00051 it the acceleration limit to go from 0 to full speed in 0.5 seconds. adjust this value for desired tuning
      output_pwm = ((acc < 0) ? -1 : 1) * 0.00051 * dt2 + prev_output_pwm;
    }
    prevTime2 = micros();
    prev_output_pwm = output_pwm;
  }
  analogWrite(pwmPin, output_pwm);
}



#endif
