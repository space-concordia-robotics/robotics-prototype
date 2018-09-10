#ifndef ROVERMOTOR_H
#define ROVERMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"

enum motor_code {MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6}; // defines 6 motors
enum motor_direction {CLOCKWISE = 1, COUNTER_CLOCKWISE}; // defines motor directions // can go into ArmMotor
enum motor_speed {SPEED0 = 1, SPEED1, SPEED2, SPEED3}; // defines motor speed // can go into ArmMotor

class RoverMotor {
  public:
    elapsedMillis sinceStart; // automatically increments every millisecond
    static int numMotors;
    //float maxCWAngle, maxCCWAngle;
    float currentAngle, desiredAngle;

    //int maxSpeed;
    int cwSpeed, ccwSpeed;
    bool movementDone;

    RoverMotor();

    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    //void setMaxCWAngle(int angle); // need to have defined it for servos first
    //void setMaxCCWAngle(int angle); // need to have defined it for servos first
    //void setMaxSpeed();
    //void setDesiredAngle(float angle); // need to have defined it for servos first
    //virtual float getCurrentAngle(void) = 0; // need to have defined it for servos first
    private:
    virtual void budge(int budgeDir, int budgeSpeed, unsigned int budgeTime)=0; // budges motor for short period of time
    virtual void setVelocity(int motorDir, int motorSpeed)=0; // sets motor speed until next timer interrupt
    
    //void update(); // can go into ArmMotor

  protected:

};

int RoverMotor::numMotors=0; // must initialize variable outside of class

RoverMotor::RoverMotor() {
  numMotors++;
}

#endif
