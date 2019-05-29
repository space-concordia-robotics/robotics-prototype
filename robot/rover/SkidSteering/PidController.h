#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PidController {
  public:
    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    PidController();
    float updatePID(volatile float currentAngle, float desiredAngle);
    void setJointVelocityTolerance(float tolerance);
    void setOutputLimits(float minVal, float maxVal, float zeroVal);
    void setGainConstants(float kp, float ki, float kd);
    float getMaxOutputValue(void);
    float getMinOutputValue(void);
    float getJointVelocityTolerance(void);
    void printPidParameters(void);
  private:
    float kp, ki, kd;
    float jointVelocityTolerance;
    float maxOutputValue, minOutputValue, slowestSpeed;
    elapsedMillis dt;
    float pTerm, iTerm, dTerm;
    float error, previousError;
    float errorSum;
    float pidSum; // pid output, must be checked before assigning this value to pidOutput
};

PidController::PidController() {
  // default values
  //    kp = 10.0;
  //    ki = 0.0;
  //    kd = 0.0;
  jointVelocityTolerance = 1.0;
  minOutputValue = -30.0;
  maxOutputValue = 30.0;
  slowestSpeed = 3.0;
}

float PidController::updatePID(volatile float currentAngle, float desiredAngle) {
  //    float pidOutput;
  error = desiredAngle - currentAngle; // these angle variables need to be obtained from the notor object
  // if the angle is outside the tolerance, move
  //  if (fabs(error) >= jointVelocityTolerance)
  //  {
  // fabs is for floats
  //    Serial.println(pidSum);
  //  Serial.print(error);
  //    Serial.print(" ");
  pTerm = kp * error;
  iTerm += ki * ((error + previousError) / 2) * dt;
  if (iTerm > 255) iTerm = 255;
  if (iTerm < 0) iTerm = 0;
  dTerm = kd * (error - previousError) / dt;
  pidSum = pTerm + iTerm + dTerm;
  //      Serial.println(pidSum);

  if (pidSum > 255) pidSum = 255;
  if (pidSum < 0) pidSum = 0;

  return pidSum;
}

void PidController::setJointVelocityTolerance(float tolerance) {
  jointVelocityTolerance = tolerance;
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

float PidController::getJointVelocityTolerance(void) {
  return jointVelocityTolerance;
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
  UART_PORT.println(jointVelocityTolerance);
#endif
}

#endif
