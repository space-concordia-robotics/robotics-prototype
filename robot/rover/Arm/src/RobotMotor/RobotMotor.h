#ifndef ROBOTMOTOR_H
#define ROBOTMOTOR_H
#include <Arduino.h>
#include "../PinSetup/PinSetup.h"
#include "../PidController/PidController.h"

#define BUDGE_TIMEOUT 200 //!< if command hasn't been received in this amount of ms, stop turning
#define FLEXION_SWITCH 'f' //!< for joints that flex/extend
#define REVOLUTE_SWITCH 'r' //!< for joints that swivel
#define GRIPPER_SWITCH 'g' //!< for gripper limit switch
#define SINGLE_ENDED_HOMING 1 //!< only home towards inside angles
#define DOUBLE_ENDED_HOMING 2 //!< home towards both directions

enum motor_direction {CLOCKWISE = -1, COUNTER_CLOCKWISE = 1}; //!< defines motor directions
enum loop_state {OPEN_LOOP = 1, CLOSED_LOOP}; //!< defines whether motor control is open loop or closed loop
enum motor_type {DC_MOTOR = 1, POSITION_SERVO, CONTINUOUS_SERVO, STEPPER_MOTOR}; //!< defines motor type

/*! \brief The base class for ServoMotor, StepperMotor and DcMotor.
 *
 * \todo Make joint objects instead of motor objects and have switches etc inside of them?
 * \todo Perhaps the initialization of motor angle parameters should be its own function
 * \todo Perhaps the initialization of the motor's pins should also be its own function
 * \todo Perhaps the constructor should take a series of structs, one for each type
 */
class RobotMotor {
  public:
    // these variables are set at start and normally don't change during the main loop
    int motorType;
    int encoderModifier; //!< flips encoder directionality in the case of flipped wires
    static int numMotors; //!< keeps track of how many motors there are
    int encoderPinA, encoderPinB;
    // for limit switch interrupts and homing
    int limSwitchCw, limSwitchCcw, limSwitchFlex, limSwitchExtend, limSwitchOpen;
    bool hasLimitSwitches;
    char limitSwitchType;
    volatile bool triggered;
    bool actualPress;
    volatile int triggerState;
    int limitSwitchState;
    elapsedMillis sinceTrigger;
    int homingType; //!< for homing
    bool homingDone; //!< for homing
    bool atSafeAngle; //!< for homing
    bool startedZeroing; //!< for homing
    int homingPass; //!< for homing
    void checkForActualPress(void);
    void goToSafeAngle(void);
    void homeMotor(char homingDir);
    void stopHoming(void);
    // other stuff
    float gearRatio, gearRatioReciprocal; //!< calculating this beforehand improves speed of floating point calculations
    float encoderResolutionReciprocal; //!< calculating this beforehand improves speed of floating point calculations
    float maxJointAngle, minJointAngle; //!< joint angle limits, used to make sure the arm doesn't bend too far and break itself
    float minHardAngle, maxHardAngle, minSoftAngle, maxSoftAngle;
    bool hasAngleLimits; //!< a wrist which wants to turn infinitely will be constrained by angle limits
    bool isOpenLoop; //!< decides whether to use the PID or not
    bool hasRamping; //!< decides whether to ramp the speed in open loop
    volatile int rotationDirection;
    // int maxSpeed;
    PidController pidController; //!< used for speed and angle control
    // these variables change during the main loop
    volatile long encoderCount; //!< incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile bool movementDone; //!< this variable is what allows the timer interrupts to make motors turn. can be updated within said interrupts
    elapsedMillis sinceBudgeCommand; //!< timeout for budge commands, elapsedMillis can't be volatile
    volatile bool isBudging;
    // setup functions
    RobotMotor();
    void attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes);
    void attachLimitSwitches(char type, int switch1, int switch2);
    void setAngleLimits(float minHardAngle, float maxHardAngle, float minSoftAngle, float maxSoftAngle); //!< sets joint limits so the arm doesn't break from trying to reach physically impossible angles
    bool withinJointAngleLimits(float angle); //!< checks if angle is within limits
    bool hasEncoder;
    // void setMaxSpeed();
    /* movement helper functions */
    int calcDirection(float error); //!< updates rotationDirection based on the angular error inputted
    bool setDesiredAngle(float angle); //!< if the angle is valid, update desiredAngle and return true. else return false.
    float getDesiredAngle(void); //!< return copy of the desired angle, not a reference to it
    virtual bool calcCurrentAngle(void) = 0;
    void setSoftwareAngle(float angle);
    float getSoftwareAngle(void);
    void switchDirectionLogic(void); //!< tells the motor to reverse the direction for a motor's control... does this need to be virtual?
    int getDirectionLogic(void); //!< returns the directionModifier;
    void setGearRatio(float ratio); //!< sets the gear ratio for the motor joint
    void setOpenLoopGain(float loopGain); //!< set gain which adjusts open loop speed for open loop angle control
    void setMotorSpeed(float motorSpeed); //!< set open loop speed and max speed of pid output
    float getMotorSpeed(void); //!< return motor speed
    /* movement functions */
    virtual void stopRotation(void) = 0; //!< stop turning the motor
    virtual void setVelocity(int motorDir, float motorSpeed) = 0; //!< sets motor speed and direction until next timer interrupt
    virtual void goToCommandedAngle(void) = 0; //!< once an angle is set it will go to the angle and won't ignore limits
    virtual void forceToAngle(float angle) = 0; //!< goes to angle ignoring limits
    virtual void budge(int dir) = 0; //!< moves a motor as long as a new budge commmand comes in within 200ms
    // for open loop control
    float openLoopError; //!< public variable for open loop control
    float startAngle; //!< used in angle esimation
  private:
    // doesn't really make sense to have any private variables for this parent class.
    // note that virtual functions must be public in order for them to be accessible from motorArray[]
  protected:
    // the following variables are specific to encoders
    uint32_t encoderPort; //!< address of the port connected to a particular encoder pin
    int encoderShift; //!< how many bits to shift over to find the encoder pin state
    int encoderResolution; //!< ticks per revolution
    volatile float currentAngle; //!< can be updated within timer interrupts
    volatile float imaginedAngle;
    float desiredAngle;
    int directionModifier;
    // for open loop control
    int openLoopSpeed; //!< angular speed (degrees/second)
    float openLoopGain; //!< speed correction factor
};



#endif
