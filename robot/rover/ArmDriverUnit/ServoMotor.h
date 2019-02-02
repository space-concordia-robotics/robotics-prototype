#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "PinSetup.h"
#include "RobotMotor.h"

class ServoMotor:public RobotMotor {
  public:
    static int numServoMotors;

    ServoMotor(int pwmPin, float gearRatio);

    bool calcTurningDuration(float angle); // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    bool calcCurrentAngle(void);
    void setVelocity(int motorDir, float motorSpeed); // currently this actually activates the servo and makes it turn at a set speed/direction
    void stopRotation(void);

    // stuff for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    float openLoopGain; // speed correction factor
    unsigned int numMillis; // how many milliseconds for servo to reach desired position
    elapsedMillis timeCount; // how long has the servo been turning for
  
  private:
    int pwmPin;
    Servo servo;
};

int ServoMotor::numServoMotors = 0; // must initialize variable outside of class

ServoMotor::ServoMotor(int pwmPin, float gearRatio):
  pwmPin(pwmPin)
  {
    servo.attach(pwmPin);
    numServoMotors++;
    // variables declared in RobotMotor require the this-> operator
    this -> gearRatio = gearRatio;
    this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
    this -> motorType = CONTINUOUS_SERVO;
    hasEncoder = false;
    
    openLoopSpeed = 0; // no speed by default;
    openLoopGain = 1.0; // temp open loop control
  }

  void ServoMotor::stopRotation(void) {
    servo.writeMicroseconds(SERVO_STOP);
    movementDone = true;
  }

  // takes a direction and offset from SERVO_STOP and sends appropriate pwm signal to servo
  void ServoMotor::setVelocity(int motorDir, float motorSpeed)
  {
    if (!isOpenLoop)
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

    // pulse time varies from 1000 to 2000, 1500 being the midpoint, so 500 is the offset from 1500
    int pulseTime = SERVO_STOP + (motorSpeed * motorDir * 500 / 100);
    if (pulseTime > 2000) pulseTime = 2000;
    if (pulseTime < 1000) pulseTime = 1000;
    servo.writeMicroseconds(pulseTime);
  }

  bool ServoMotor::calcTurningDuration(float angle) {
    // if the error is big enough to justify movement
    // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
    if (fabs(angle) > pidController.getJointAngleTolerance()) {
      numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
      // Serial.println(numMillis);
      return true;
    }
    else {
      return false;
    }
  }

  bool ServoMotor::calcCurrentAngle(void) {
    if (isOpenLoop) {
      static unsigned int prevTime = 0;
      if (timeCount < numMillis) {
        // if the motor is moving, calculate the angle based on how long it's been turning for
        imaginedAngle += (float)rotationDirection*(timeCount - prevTime) * (openLoopSpeed * gearRatioReciprocal) / (1000.0 * openLoopGain);
        prevTime = timeCount;
      }
      else {
        prevTime = 0;
        // imaginedAngle hasn't changed since motor hasn't moved and encoder isn't working
      }
      return true;
    }
    else
      if (hasEncoder) {
        currentAngle = (float) encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
        imaginedAngle = currentAngle;
        return true;
      }
    else {
      return false;
    }
  }

#endif
