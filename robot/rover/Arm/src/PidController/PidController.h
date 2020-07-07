
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <Arduino.h>

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


#endif
