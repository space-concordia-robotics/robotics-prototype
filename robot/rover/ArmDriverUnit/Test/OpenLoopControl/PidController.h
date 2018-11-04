#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PidController {
  public:
    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    float jointAngleTolerance;
    float maxOutputValue;
    float minOutputValue;
    int kp;
    int ki;
    int kd;

    float pidOutput; // only updated at the end of the calculations

    PidController();
    void updatePID(volatile float& currentAngle, float& desiredAngle);
    void setJointAngleTolerance(float tolerance);
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
  jointAngleTolerance = 1.0;
  minOutputValue = -50.0;
  maxOutputValue = 50.0;
}

void PidController::updatePID(volatile float& currentAngle, float& desiredAngle) {
  error = desiredAngle - currentAngle; // these angle variables need to be obtained from the notor object
  // if the angle is outside the tolerance, move
  if (fabs(error) >= jointAngleTolerance) { //fabs is for floats

    Serial.print("Current angle is: "); Serial.println(currentAngle);
    Serial.print("Desired angle is: "); Serial.println(desiredAngle);
    Serial.print("Previous angle error is: "); Serial.println(previousError);
    Serial.print("Current angle error is: "); Serial.println(error);
    pTerm = kp * error;
    Serial.print("P output is: "); Serial.println(pTerm);
    iTerm += ki * ((error + previousError) / 2) * dt;
    Serial.print("I output is: "); Serial.println(iTerm);
    dTerm = kd * (error - previousError) / dt;
    Serial.print("D output is: "); Serial.println(dTerm);
    pidSum = pTerm + iTerm + dTerm;
    Serial.print("PID output is: "); Serial.println(pidSum);

    if (pidSum > maxOutputValue) pidOutput = maxOutputValue; // give max output
    if (pidSum < minOutputValue) pidOutput = minOutputValue; // give min output // do i need to check for speeds that are too slow? probz
    else pidOutput = pidSum;
  }
  // if the angle is within the tolerance, don't move
  else pidOutput = 0; // speed is 0

  // prepare for next pid call
  previousError = error;
  dt = 0; // Reset timer
}

void PidController::setJointAngleTolerance(float tolerance) {
  jointAngleTolerance = tolerance;
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
