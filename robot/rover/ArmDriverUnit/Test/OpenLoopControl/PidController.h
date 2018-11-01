#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PidController {
  public:
    // these variables are accessed outside of ISRs
    //volatile int dir; // closed loop direction
    volatile float pidOutput; // only updated at the end of the calculations

    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    float angleTolerance;
    float maxOutputValue;
    float minOutputValue;
    int kp;
    int ki;
    int kd;

    PidController();
    void updatePID(volatile float& currentAngle, float& desiredAngle);
    void setAngleTolerance(float tolerance);
    void setOutputLimits(float minVal, float maxVal);
    void setGainConstants(float kp, float ki, float kd);

  private:
    elapsedMillis dt;
    float pTerm, iTerm, dTerm;
    float error;
    float previousError;
    float errorSum;
    float pidSum; // pid output, must be checked before assigning this value to pidOutput
};

PidController::PidController() {
  // default values
  kp = 1.0; ki = 0.0; kd = 0.0;
  angleTolerance = 2.0;
  minOutputValue = -50.0;
  maxOutputValue = 50.0;
}

void PidController::updatePID(volatile float& currentAngle, float& desiredAngle) {

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
    //if (pidSum > 0) dir = 1; else dir = -1; // positive dir is ccw, negative is cw
    //Serial.print("direction is: "); Serial.println(dir);
    if (pidSum > maxOutputValue) pidOutput = maxOutputValue; // give max output
    if (pidSum < minOutputValue) pidOutput = minOutputValue; // give min output // do i need to check for speeds that are too slow? probz
    else pidOutput = pidSum;

    // set motor speed somewhere

    //Reset error
    previousError = error;
    dt = 0; //sincePID=0;
  }
  // if the angle is within the tolerance, don't move
  else pidOutput = 0; // speed is 0
}

void PidController::setAngleTolerance(float tolerance) {
  angleTolerance = tolerance;
}

void PidController::setOutputLimits(float minVal, float maxVal) {
  maxOutputValue = maxVal;
  minOutputValue = minVal;
}

void PidController::setGainConstants(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

#endif
