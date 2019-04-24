
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

/*! \brief This is intended to be a generic PID controller class.
 * 
 * As it stands, however, it is built with position control in mind.
 * It holds modifiable P, I and D gains and limits the output
 * to prevent saturation. The output ranges from -100.0% to 100.0%
 * but by default it is limited to +/-50%. There is also a deadband
 * range around 0% which can be used in the case of position control
 * to stop a motor from whining when being sent pwm values close to 0.
 * Finally, the PID controller ignores errors smaller than a certain
 * resolution which can be defined by the user. This is useful for
 * discrete plants such as stepper motors, or even continuous plants
 * for which small angle errors can be ignored.
 * 
 * \todo Can I deal with the fact that angleTolerance is a PidController attribute and not a RobotMotor attribute?
 */
class PidController {
  public:
    PidController();
    float updatePID(volatile float currentAngle, float desiredAngle);
    void setJointAngleTolerance(float tolerance);
    void setOutputLimits(float minVal, float maxVal);
    void setOutputLimits(float minVal, float maxVal, float zeroVal);
    void setSlowestSpeed(float minSpeed);
    void setGainConstants(float kp, float ki, float kd);
    float getMaxOutputValue(void);
    float getMinOutputValue(void);
    float getJointAngleTolerance(void);
    void printPidParameters(void);
  private:
    float kp, ki, kd; //!< PID gains
    float jointAngleTolerance; //!< minimum joint angle for which the PID will allow calculations
    float maxOutputValue, minOutputValue; //!< max and min outputs which limit the output below 100%
    float slowestSpeed; //!< this defines the deadband around 0%
    elapsedMillis dt; //!< how long since the previous PID calculation
    float pTerm, iTerm, dTerm; //!< PID terms before they are summed together
    float error; //!< difference between measured point and setpoint
    float previousError; //!< used in derivative calulation
    float errorSum; //!< used in integral calculation
    float pidSum; //!< this is an intermediate value used to check if the output is saturating
    float pidOutput; //!< the actual output of the PID controller after saturation has been accounted for  
};

/*! \brief Constructor. Sets kp to 1, ki and kd to 0. Sets jointAngleTolerance
 * to 1 degree and limits the output to 50%. slowestSpeed is set to 3%.
 */
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

/*! \brief Calculates and returns the PID output based on the setpoint and measured point.
 * If the error is not so small that the motor can't move to it,
 * the P, I and D terms are calculated. The sum is checked and
 * limited if it exceeds the output limits. If the sum is smaller
 * than the minimum allowable motor speed, the output is set to 0.
 * The result is returned directly but also saved inside the class.
 * @param[in] currentAngle The current angle based on encoder values or open loop estimations
 * @param[in] desiredAngle The desired angle set by the motor command
 * @param[out] pidOutput   The PID output
 */
float PidController::updatePID(volatile float currentAngle, float desiredAngle) {
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

//! Set the PID gains
void PidController::setGainConstants(float kp, float ki, float kd) {
  this -> kp = kp;
  this -> ki = ki;
  this -> kd = kd;
}

//! Set the minimum angle that the motor will move for
void PidController::setJointAngleTolerance(float tolerance) {
  jointAngleTolerance = tolerance;
}

//! Return the minimum angle that the motor will move for
float PidController::getJointAngleTolerance(void) {
  return jointAngleTolerance;
}

//! Set the min and max outputs, represented as speed for motors. This version does not take the deadband into consideration.
void PidController::setOutputLimits(float minVal, float maxVal) {
  maxOutputValue = maxVal;
  minOutputValue = minVal;
}

//! Set the min and max outputs, represented as speed for motors. This version takes the deadband into consideration.
void PidController::setOutputLimits(float minVal, float maxVal, float zeroVal) {
  maxOutputValue = maxVal;
  minOutputValue = minVal;
  slowestSpeed = zeroVal;
}

//! Below this speed output from the pid, the motor will simply stop turning to avoid noise
void PidController::setSlowestSpeed(float minSpeed){
  slowestSpeed = minSpeed;
}

//! Return the max output value
float PidController::getMaxOutputValue(void) {
  return maxOutputValue;
}

//! Return the min output value
float PidController::getMinOutputValue(void) {
  return minOutputValue;
}

//! Print the PID gains using serial communication
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
