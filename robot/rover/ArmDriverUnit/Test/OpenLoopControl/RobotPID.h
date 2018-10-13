#ifndef ROBOTPID_H
#define ROBOTPID_H

class RobotPID {
  public:
    //temp stuff for open loop control
    int openLoopDir; // public variable for open loop control of steppers and servos
    float openLoopError; // public variable for open loop control of steppers and servos
    int numSteps; // how many steps to take for stepper to reach desired position
    unsigned int numMillis; // how many milliseconds for servo to reach desired position
    int openLoopSpeed; // angular speed of servo (degrees/second)
    volatile int stepCount; // how many steps the stepper has taken since it started moving
    elapsedMillis timeCount; // how long has the servo been turning for
    float openLoopGain; // multiplier so that speed calculation works for given speed

    // these variables are accessed outside of ISRs
    volatile int dir; // closed loop direction
    volatile float pidOutput; // only updated at the end of the calculations
    int motorSpeed; // not even used yet

    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    float angleTolerance;
    float maxOutputValue;
    float minOutputValue;
    int kp;
    int ki;
    int kd;

    RobotPID();
    void updatePID(volatile float& currentAngle, float& desiredAngle);
    void setAngleTolerance(float tolerance);
    void setOutputLimits(float minVal, float maxVal);
    void setGainConstants(float kp, float ki, float kd);

  private:
    //unsigned int dt; // period at which the pid is called
    elapsedMillis dt;//sincePID;
    float pTerm, iTerm, dTerm;
    float error;
    float previousError;
    float errorSum;
    float pidSum; // pid output, must be checked before assigning this value to pidOutput
};

RobotPID::RobotPID() {
  //movementDone = true;
  // default values
  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
  kp = 1.0; ki = 0.0; kd = 0.0;
  angleTolerance = 2.0;
  minOutputValue = 0.0;
  maxOutputValue = 50.0;
}

void RobotPID::updatePID(volatile float& currentAngle, float& desiredAngle) {

  error = desiredAngle - currentAngle; // these angle variables need to be obtained from the notor object
  // if the angle is outside the tolerance, move
  if (fabs(error) > angleTolerance) { //fabs is for floats i think?

    Serial.print("Current angle is: "); Serial.println(currentAngle);
    Serial.print("Desired angle is: "); Serial.println(desiredAngle);
    Serial.print("Previous angle error is: "); Serial.println(previousError);
    Serial.print("Current angle error is: "); Serial.println(error);
    Serial.print("dt is: "); Serial.println(dt);
    pTerm = kp * error;
    Serial.print("P output is: "); Serial.println(pTerm);
    iTerm += ki * ((error + previousError) / 2) * dt;
    //integralTerm += (ki[i] * error * timeChange);
    //_integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
    Serial.print("I output is: "); Serial.println(iTerm);
    dTerm = kd * (error - previousError) / dt;
    //derivativeTerm = kd[i]* (motorAngle[i] - lastInput) / timeChange;
    //double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
    Serial.print("D output is: "); Serial.println(dTerm);
    pidSum = pTerm + iTerm + dTerm;
    Serial.print("PID output is: "); Serial.println(pidSum);
    if (pidSum > 0) dir = 1; else dir = -1; // positive dir is ccw, negative is cw
    Serial.print("direction is: "); Serial.println(dir);
    if (fabs(pidSum) > maxOutputValue) pidOutput = maxOutputValue; // give max output
    if (fabs(pidSum) < minOutputValue) pidOutput = minOutputValue; // give min output // do i need to check for speeds that are too slow? probz
    else pidOutput = pidSum;

    // set motor speed somewhere

    //Reset error
    previousError = error;
    dt = 0; //sincePID=0;
  }
  // if the angle is within the tolerance, don't move
  else ; // stop motor?
}

void RobotPID::setAngleTolerance(float tolerance) {
  angleTolerance = tolerance;
}

void RobotPID::setOutputLimits(float minVal, float maxVal) {
  maxOutputValue = maxVal;
  minOutputValue = minVal;
}

void RobotPID::setGainConstants(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

#endif
