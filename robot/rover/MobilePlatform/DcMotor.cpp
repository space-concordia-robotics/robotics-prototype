#include "DcMotor.h"

DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio, String motorName):// if no encoder
        directionPin(dirPin), pwmPin(pwmPin)
{
    // variables declared in RobotMotor require the this-> operator
    this -> gearRatio = gearRatio;
    this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
    this -> motorName = motorName;
}


void DcMotor::calcCurrentVelocity() {
//    Serial.print(dt);
//    Serial.print(" ");
//    Serial.print(encoderCount);
//    Serial.println(" ");

    if (dt <= 0 || encoderCount <= 0) {
        currentVelocity = 0;
    }
    else {
        currentVelocity = (float) (encoderCount * 60000000.0 * gearRatioReciprocal * encoderResolutionReciprocal / (float) (dt));
    }

    encoderCount = 0;
    dt = 0;
    //  Serial.println(dt/1000);
}

float DcMotor::getCurrentVelocity(void) {
    return currentVelocity;
}

void DcMotor::setVelocity(int motorDir, float dV, volatile float currentVelocity) {

    desiredVelocity = dV;
    desiredDirection = motorDir;
//    Serial.print(motorName);
    switch (motorDir) {
        case CW:
            digitalWrite(directionPin, HIGH);
            break;
        case CCW:
            digitalWrite(directionPin, LOW);
            break;
    }

    if (isOpenLoop) {

        output_pwm = desiredVelocity;
        analogWrite(pwmPin, output_pwm);

    }
    else if (!isOpenLoop) {
        // THIS LOOKS WRONG
        // makes sure the speed is within the limits set in the pid during setup
        if (desiredVelocity > 30) {
            desiredVelocity = 30;
        }
        else if (desiredVelocity < 0) {
            desiredVelocity = 0;
        }
        output_pwm = pidController.updatePID(currentVelocity, desiredVelocity);

//        Serial.print(output_pwm);
//        Serial.print(" ");

        analogWrite(pwmPin, fabs(output_pwm));

    }
    //this -> desiredVelocity = output_pwm;

    /* Acceleration limiter */
//    if (accLimit) {
//        final_output_pwm = output_pwm;
//        dt2 = micros() - prevTime2;
//
//        acc = ((float) output_pwm - (float) prev_output_pwm) / (float) dt2;
//
//        if (abs(acc) > 0.00051) {  // 0.00051 it the acceleration limit to go from 0 to full speed in 0.5 seconds. adjust this value for desired tuning
//            output_pwm = ((acc < 0) ? -1 : 1) * 0.000151 * dt2 + prev_output_pwm;
//        }
//        prevTime2 = micros();
//        prev_output_pwm = output_pwm;
//        analogWrite(pwmPin, output_pwm);
//    }
//    else analogWrite(pwmPin, output_pwm);

}

void DcMotor::velocityHandler(DcMotor* motors, float throttle, float steering) {
  // steering of 0 means both motors at throttle.
  // as steering moves away from 0, one motor keeps throttle.
  // the other slows down or even reverses based on the steering value.
  // multiplier is mapped between -1 and 1 so 0 means no speed and the extremes mean full throttle in either direction.

  float multiplier = Helpers::get().mapFloat(fabs(steering), 0, MAX_INPUT_VALUE, 1, -1);
  float leadingSideAbs = Helpers::get().mapFloat(fabs(throttle), 0, MAX_INPUT_VALUE, 0, maxOutputSignal);
  float trailingSideAbs = leadingSideAbs * multiplier;
  float desiredVelocityLeft = 0.0;
  float desiredVelocityRight = 0.0;
  
  int dir = 1;
  if (throttle >= 0) dir = 1;
  else if (throttle < 0) dir = -1;

  if (steering < 0) { // turning left
    desiredVelocityRight = leadingSideAbs * dir;
    desiredVelocityLeft = trailingSideAbs * dir;
  }
  else { // turning right
    desiredVelocityRight = trailingSideAbs * dir;
    desiredVelocityLeft = leadingSideAbs * dir;
  }

  if (desiredVelocityLeft > 0) leftMotorDirection = CCW;
  else leftMotorDirection = CW;
  if (desiredVelocityRight < 0) rightMotorDirection = CCW;
  else rightMotorDirection = CW;

  motors[0].setVelocity(rightMotorDirection, fabs(desiredVelocityRight), motors[0].getCurrentVelocity());
  motors[1].setVelocity(rightMotorDirection, fabs(desiredVelocityRight), motors[1].getCurrentVelocity());
  motors[2].setVelocity(rightMotorDirection, fabs(desiredVelocityRight), motors[2].getCurrentVelocity());
  motors[3].setVelocity(leftMotorDirection, fabs(desiredVelocityLeft), motors[3].getCurrentVelocity());
  motors[4].setVelocity(leftMotorDirection, fabs(desiredVelocityLeft), motors[4].getCurrentVelocity());
  motors[5].setVelocity(leftMotorDirection, fabs(desiredVelocityLeft), motors[5].getCurrentVelocity());
}
