#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotMotor.h"

// for 3.3v output
// pwm speed control
//#define SERVO_STOP 189 // motor is supposed to stop at 50% duty cycle (127/255)
// speed 0, slowest
#define SERVO_CW1 SERVO_STOP+15
#define SERVO_CCW1 SERVO_STOP-15
// speed 1
#define SERVO_CW2 SERVO_STOP+30
#define SERVO_CCW2 SERVO_STOP-30
// speed 2
#define SERVO_CW3 SERVO_STOP+50
#define SERVO_CCW3 SERVO_STOP-50
// speed 3, fastest
#define SERVO_CW4 255
#define SERVO_CCW4 SERVO_STOP-(255-SERVO_STOP)

const int servoCwSpeedArray[] = {SERVO_CW1, SERVO_CW2, SERVO_CW3, SERVO_CW4};
const int servoCcwSpeedArray[] = {SERVO_CCW1, SERVO_CCW2, SERVO_CCW3, SERVO_CCW4};

// for 5v output
/*// pwm speed control
  //#define SERVO_STOP 127 // motor is supposed to stop at 50% duty cycle (127/255)
  // speed 0, slowest
  #define SERVO_CW0 SERVO_STOP+32
  #define SERVO_CCW0 SERVO_STOP-32
  // speed 1
  #define SERVO_CW1 SERVO_STOP+64
  #define SERVO_CCW1 SERVO_STOP-64
  // speed 2
  #define SERVO_CW2 SERVO_STOP+96
  #define SERVO_CCW2 SERVO_STOP-96
  // speed 3, fastest
  #define SERVO_CW3 255
  #define SERVO_CCW3 1
*/

class ServoMotor : public RobotMotor {
  public:
    static int numServoMotors;

    ServoMotor(int pwmPin, float gearRatio);

    // budges motor for short period of time
    void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED,
               unsigned int budgeTime = DEFAULT_BUDGE_TIME);
    void setVelocity(int motorDir, int motorSpeed);
    void stopRotation(void);
    //float calcCurrentAngle();

    // stuff for open loop control
    int openLoopDirection; // public variable for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    float openLoopGain; // speed correction factor
    unsigned int numMillis; // how many milliseconds for servo to reach desired position
    elapsedMillis timeCount; // how long has the servo been turning for

  private:
    int pwmPin;
};

int ServoMotor::numServoMotors = 0; // must initialize variable outside of class

ServoMotor::ServoMotor(int pwmPin, float gearRatio):
  pwmPin(pwmPin)
{
  numServoMotors++;
  this->gearRatio = gearRatio;
  this->gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  hasEncoder = false;
  budgeMovementDone = true;

  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
}

void ServoMotor::budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) {
  if (budgeDir <= COUNTER_CLOCKWISE && budgeSpeed > 0 && budgeSpeed <= MAX_SPEED
      && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
    if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
      canTurnRight = true; Serial.println("preparing to turn servo clockwise");
      rightCount++; leftCount--;
    }
    else if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
      canTurnLeft = true; Serial.println("preparing to turn servo counter-clockwise");
      leftCount++; rightCount--;
    }
    else Serial.println("max turn count reached");
    Serial.print("right servo count "); Serial.println(rightCount);
    Serial.print("left servo count "); Serial.println(leftCount);

    if (budgeDir == CLOCKWISE && canTurnRight) {
      budgeMovementDone = false;
      cwSpeed = servoCwSpeedArray[budgeSpeed];
      ccwSpeed = servoCcwSpeedArray[budgeSpeed];
      Serial.print("setting servo speed level to "); Serial.println(budgeSpeed); Serial.println("starting servo movement");
      sinceStart = 0;
      analogWrite(pwmPin, cwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, SERVO_STOP); // sets duty cycle to 50% which corresponds to 0 speed
      budgeMovementDone = true; Serial.println("servo movement done");
    }
    if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
      budgeMovementDone = false;
      cwSpeed = servoCwSpeedArray[budgeSpeed];
      ccwSpeed = servoCcwSpeedArray[budgeSpeed];
      Serial.print("setting servo speed level to "); Serial.println(budgeSpeed);
      sinceStart = 0;
      analogWrite(pwmPin, ccwSpeed);
      while (sinceStart < budgeTime) ; // wait
      analogWrite(pwmPin, SERVO_STOP); // sets duty cycle to 50% which corresponds to 0 speed
      budgeMovementDone = true; Serial.println("servo movement done");
    }
    canTurnRight = false; canTurnLeft = false;
  }
}

void ServoMotor::stopRotation(void) {
  analogWrite(pwmPin, SERVO_STOP);
}

// takes a direction and offset from SERVO_STOP and sends appropriate pwm signal to servo
void ServoMotor::setVelocity(int motorDir, int motorSpeed) {
  int dutyCycle;
  if (motorSpeed > motorPID.maxOutputValue) motorSpeed = motorPID.maxOutputValue;
  if (motorSpeed < motorPID.minOutputValue) motorSpeed = motorPID.minOutputValue;
  switch (motorDir) {
    case CLOCKWISE:
      dutyCycle = SERVO_STOP + motorSpeed * 128 / 100;
      break;
    case COUNTER_CLOCKWISE:
      dutyCycle = SERVO_STOP - motorSpeed * 128 / 100;
      break;
    default:
      dutyCycle = 0;
  }
  if (dutyCycle > 255) dutyCycle = 255;
  if (dutyCycle < 0) dutyCycle = 0;

  analogWrite(pwmPin, dutyCycle);
}

#endif
