#ifndef ROBOTPID_H
#define ROBOTPID_H

class RobotPID {
  public:
    int dir;
    int motorSpeed;
    float angleTolerance; // arbitrary value, redefine this elsewhere, may want to define one per motor
    float maxOutputValue; // arbitrary value, redefine this elsewhere, may want to define one per motor
    float minOutputValue; // arbitrary value, redefine this elsewhere, may want to define one per motor

    int kp, ki, kd;
    int dt; // period at which the pid is called
    float pTerm, iTerm, dTerm;

    float error;
    float previousError;
    float errorSum;
    float pidOutput;

    void updatePID(float& currentAngle, float& desiredAngle);
  private:
  protected:

};

void RobotPID::updatePID(float& currentAngle, float& desiredAngle) {

  error = desiredAngle - currentAngle; // these angle variables need to be obtained from the notor object
  // if the angle is outside the tolerance, move
  if (fabs(error) > angleTolerance) { //fabs is for floats i think?

    pTerm = kp * error;
    iTerm += ki * ((error + previousError) / 2) * dt;
    //integralTerm += (ki[i] * error * timeChange);
    //_integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
    dTerm = kd * (error - previousError) / dt;
    //derivativeTerm = kd[i]* (motorAngle[i] - lastInput) / timeChange;
    //double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
    pidOutput = pTerm + iTerm + dTerm;
    if (pidOutput > 0) dir = 1; else dir = -1; // positive dir is ccw, negative is cw
    if (fabs(pidOutput) > maxOutputValue) pidOutput = maxOutputValue; // give max output
    if (fabs(pidOutput) < minOutputValue) pidOutput = minOutputValue; // give min output // do i need to check for speeds that are too slow? probz

    // set motor speed somewhere

    //Reset error
    previousError = error;
  }
  // if the angle is within the tolerance, don't move
  else ; // stop motor?
}

#endif
