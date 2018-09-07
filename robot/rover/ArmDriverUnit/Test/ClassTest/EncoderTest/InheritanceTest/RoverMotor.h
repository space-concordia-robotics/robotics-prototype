#ifndef ROVERMOTOR_H
#define ROVERMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"

enum motor_code {MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6}; // defines 6 motors
enum motor_direction {CLOCKWISE, COUNTER_CLOCKWISE}; // defines motor directions // can go into ArmMotor
enum motor_speed {SPEED0, SPEED1, SPEED2, SPEED3}; // defines motor speed // can go into ArmMotor

class RoverMotor {
  public:
    elapsedMillis sinceStart; // automatically increments every millisecond

    //float maxCWAngle, maxCCWAngle;
    //float currentAngle, desiredAngle;

    //int maxSpeed;
    int cwSpeed, ccwSpeed;
    bool movementDone;

    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    //void setMaxCWAngle(int angle);
    //void setMaxCCWAngle(int angle);
    //void setMaxSpeed();
    //void setDesiredAngle(float angle);
    //float getCurrentAngle();

    // budges motor for short period of time
    virtual void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);

    //void update(); // can go into ArmMotor

  protected:

};

#endif
