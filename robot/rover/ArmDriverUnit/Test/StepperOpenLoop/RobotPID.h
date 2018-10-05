#ifndef ROBOTPID_H
#define ROBOTPID_H

class RobotPID {
  public:
    //temp stuff for stepper open loop control
    int openLoopDir;
    float openLoopError;
    int numSteps;
    volatile int stepCount;

    // these variables are accessed outside of ISRs
    //volatile bool movementDone;
    volatile int dir;
    volatile float pidOutput; // only updated at the end of the calculations
    int motorSpeed; // not even used yet

    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    float angleTolerance = 2.0;
    float maxOutputValue = 300.0;
    float minOutputValue = 15.0;
    int kp = 1;
    int ki = 0;
    int kd = 0;

    RobotPID();
    void updatePID(volatile float& currentAngle, float& desiredAngle);

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

#endif
