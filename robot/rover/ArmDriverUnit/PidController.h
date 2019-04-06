
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PidController {
  public:
    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    PidController();
    float updatePID(volatile float currentAngle, float desiredAngle);
    void setJointAngleTolerance(float tolerance);
    void setOutputLimits(float minVal, float maxVal);
    void setOutputLimits(float minVal, float maxVal, float zeroVal);
    void setGainConstants(float kp, float ki, float kd);
    float getMaxOutputValue(void);
    float getMinOutputValue(void);
    float getJointAngleTolerance(void);
    void printPidParameters(void);
  private:
    float kp, ki, kd;
    float jointAngleTolerance;
    float maxOutputValue, minOutputValue, slowestSpeed;
    elapsedMillis dt;
    float pTerm, iTerm, dTerm;
    float error, previousError;
    float errorSum;
    float pidSum; // pid output, must be checked before assigning this value to pidOutput
};

PidController::PidController() {
  // default values
  kp = 1.0;
  ki = 0.0;
  kd = 0.0;
  jointAngleTolerance = 1.0;
  minOutputValue = -50.0;
  maxOutputValue = 50.0;
  slowestSpeed = 3.0;
}

float PidController::updatePID(volatile float currentAngle, float desiredAngle) {
  float pidOutput;
  error = desiredAngle - currentAngle; // these angle variables need to be obtained from the notor object
  // if the angle is outside the tolerance, move
  if (fabs(error) >= jointAngleTolerance)  {
    // fabs is for floats
    pTerm = kp * error;
    iTerm += ki * ((error + previousError) / 2) * dt;
    dTerm = kd * (error - previousError) / dt;
    pidSum = pTerm + iTerm + dTerm;
#ifdef DEBUG_PID
    // UART_PORT.print("P constant is: "); UART_PORT.println(this->kp);
    // UART_PORT.print("I constant is: "); UART_PORT.println(this->ki);
    // UART_PORT.print("D constant is: "); UART_PORT.println(this->kd);
    UART_PORT.print("Current angle is: ");
    UART_PORT.println(currentAngle);
    UART_PORT.print("Desired angle is: ");
    UART_PORT.println(desiredAngle);
    // UART_PORT.print("Previous angle error is: "); UART_PORT.println(previousError);
    UART_PORT.print("Current angle error is: ");
    UART_PORT.println(error);
    UART_PORT.print("P output is: ");
    UART_PORT.println(pTerm);
    UART_PORT.print("I output is: ");
    UART_PORT.println(iTerm);
    UART_PORT.print("D output is: ");
    UART_PORT.println(dTerm);
    UART_PORT.print("PID output is: ");
    UART_PORT.println(pidSum);
#endif

    if (pidSum > maxOutputValue)    {
      pidOutput = maxOutputValue; // give max output
    }
    else if (pidSum < minOutputValue)    {
      pidOutput = minOutputValue; // give min output
    }
    else if (fabs(pidSum) < slowestSpeed)    {
      pidOutput = 0; // check for speeds that are too slow
    }
    else    {
      pidOutput = pidSum;
    }
  }
  // if the angle is within the tolerance, don't move
  else  {
    pidOutput = 0; // speed is 0
  }
  return pidOutput;
  // prepare for next pid call
  previousError = error;
  //Serial.println(pidOutput);
  dt = 0; // Reset timer
}

void PidController::setJointAngleTolerance(float tolerance) {
  jointAngleTolerance = tolerance;
}

void PidController::setOutputLimits(float minVal, float maxVal) {
  maxOutputValue = maxVal;
  minOutputValue = minVal;
}
void PidController::setOutputLimits(float minVal, float maxVal, float zeroVal) {
  maxOutputValue = maxVal;
  minOutputValue = minVal;
  slowestSpeed = zeroVal;
}

float PidController::getMaxOutputValue(void) {
  return maxOutputValue;
}

float PidController::getMinOutputValue(void) {
  return minOutputValue;
}

float PidController::getJointAngleTolerance(void) {
  return jointAngleTolerance;
}

void PidController::setGainConstants(float kp, float ki, float kd) {
  this -> kp = kp;
  this -> ki = ki;
  this -> kd = kd;
}

void PidController::printPidParameters(void) {
#ifdef DEBUG_PID
  UART_PORT.print("P constant is: ");
  UART_PORT.println(kp);
  UART_PORT.print("I constant is: ");
  UART_PORT.println(ki);
  UART_PORT.print("D constant is: ");
  UART_PORT.println(kd);
  UART_PORT.print("Max output is: ");
  UART_PORT.println(maxOutputValue);
  UART_PORT.print("Min output is: ");
  UART_PORT.println(minOutputValue);
  UART_PORT.print("Min speed is: ");
  UART_PORT.println(slowestSpeed);
  UART_PORT.print("Joint angle tolerance is: ");
  UART_PORT.println(jointAngleTolerance);
#endif
}

#endif
