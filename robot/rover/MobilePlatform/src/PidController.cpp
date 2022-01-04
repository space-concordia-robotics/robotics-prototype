#include "PidController.h"

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

float PidController::updatePID(volatile float currentVelocity, float desiredVelocity) {
    error = desiredVelocity - currentVelocity; // these angle variables need to be obtained from the motor object

//    error = fabs(error);
//    Serial.print("error ");
//    Serial.print(error);
//    Serial.print(" dt ");
//    Serial.print(dt);
//    Serial.print(" kp ");
//    Serial.print(kp);
//    Serial.print(" ki ");
//    Serial.print(ki);
//    Serial.print(" pidsum ");
    pTerm = kp * error;
//    iTerm += ki * error ;
    iTerm += ki * ((error + previousError) / 2) * dt;

    if (iTerm > 255) iTerm = 255;
    if (iTerm < 0) iTerm = 0;
    dTerm = kd * (error - previousError) / dt;
//    dTerm = kd * (error - previousError) ;
    pidSum = pTerm + iTerm + dTerm;
//    Serial.print(pidSum);

//    Serial.print(" ");

//    pidSum = constrain(pidSum, 0, 255);

    if (pidSum > 255) pidSum = 255;
  if (pidSum < 15) pidSum = 0;
//    Serial.println(pidSum);
    previousError = error;
    dt = 0;
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
