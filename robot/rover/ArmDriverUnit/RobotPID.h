#ifndef ROBOTPID_H
#define ROBOTPID_H

class RobotPID {
  public:
    int dir;
    int motorSpeed;

    // all of these values are arbitrary and should be set from outside the object
    float angleTolerance = 2.0;
    float maxOutputValue = 300.0;
    float minOutputValue = 15.0;

    int kp=10;
    int ki=1;
    int kd=2;
    //unsigned int dt; // period at which the pid is called
    elapsedMillis dt;//sincePID;
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

    Serial.print("Current angle is: ");Serial.println(currentAngle);
    Serial.print("Desired angle is: ");Serial.println(desiredAngle);
    Serial.print("Previous angle error is: ");Serial.println(previousError);
    Serial.print("Current angle error is: ");Serial.println(error);
    Serial.print("dt is: ");Serial.println(dt);
    pTerm = kp * error;
    Serial.print("P output is: ");Serial.println(pTerm);
    iTerm += ki * ((error + previousError) / 2) * dt;
    //integralTerm += (ki[i] * error * timeChange);
    //_integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
    Serial.print("I output is: ");Serial.println(iTerm);
    dTerm = kd * (error - previousError) / dt;
    //derivativeTerm = kd[i]* (motorAngle[i] - lastInput) / timeChange;
    //double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
    Serial.print("D output is: ");Serial.println(dTerm);
    pidOutput = pTerm + iTerm + dTerm;
    Serial.print("PID output is: ");Serial.println(pidOutput);
    if (pidOutput > 0) dir = 1; else dir = -1; // positive dir is ccw, negative is cw
    Serial.print("direction is: ");Serial.println(dir);
    if (fabs(pidOutput) > maxOutputValue) pidOutput = maxOutputValue; // give max output
    if (fabs(pidOutput) < minOutputValue) pidOutput = minOutputValue; // give min output // do i need to check for speeds that are too slow? probz

    // set motor speed somewhere

    //Reset error
    previousError = error;
    dt=0;//sincePID=0;
  }
  // if the angle is within the tolerance, don't move
  else ; // stop motor?
}

#endif
