#ifndef ROBOTMOTOR_H
#define ROBOTMOTOR_H
#include <Arduino.h>
#include "PinSetup.h"
#include "PidController.h"

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
        int calcDirection(bool dir); //!< updates rotationDirection based on boolean provided. True = CW/Flex, False = CCW/Extend
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
        virtual void budge(bool dir) = 0; //!< moves a motor as long as a new budge commmand comes in within 200ms
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

int RobotMotor::numMotors = 0; // must initialize variable outside of class

RobotMotor::RobotMotor() {
    numMotors++;
    movementDone = true; // by default the movement has finished so the motors don't need to move
    hasAngleLimits = false; // only set to true when setAngleLimits() is called
    isOpenLoop = true; // by default don't use PID
    hasRamping = false; // by default don't ramp the speed
    imaginedAngle = 0.0;
    rotationDirection = 0; // by default invalid value
    directionModifier = 1; // this flips the direction sign if necessary;
    isBudging = false;
    hasRamping = false;
    hasLimitSwitches = false;
    homingType = SINGLE_ENDED_HOMING;
    homingDone = true;
    setMotorSpeed(30); // 30% speed by default;
    openLoopGain = 0.005; // temp open loop control
    homingPass = 0;
    atSafeAngle = true;
    startedZeroing = false;
    encoderModifier = 1;
}

void RobotMotor::attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes) // :
    // encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{
    hasEncoder = true;
    encoderPinA = encA;
    encoderPinB = encB;
    encoderPort = port;
    encoderShift = shift;
    encoderResolution = encRes;
    encoderResolutionReciprocal = 1 / (float) encRes;
}

void RobotMotor::attachLimitSwitches(char type, int switch1, int switch2) {
    hasLimitSwitches = true;
    limitSwitchType = type;
    if (type == FLEXION_SWITCH) {
        limSwitchFlex = switch1;
        limSwitchExtend = switch2;
    }
    else if (type == REVOLUTE_SWITCH) {
        limSwitchCw = switch1;
        limSwitchCcw = switch2;
    }
    else if (type == GRIPPER_SWITCH) {
        limSwitchOpen = switch1;
    }
}

void RobotMotor::setAngleLimits(float minH, float maxH, float minS, float maxS) {
    minJointAngle = minS;
    maxJointAngle = maxS;
    minHardAngle = minH;
    maxHardAngle = maxH;
    minSoftAngle = minS;
    maxSoftAngle = maxS;
    hasAngleLimits = true;
}

bool RobotMotor::withinJointAngleLimits(float angle) {
    if (!hasAngleLimits || (angle >= minJointAngle && angle <= maxJointAngle)) {
        Serial.println("true");
        return true;
    }
    else {
        Serial.println("false");
        return false;
    }
}

bool RobotMotor::setDesiredAngle(float angle) {
    //! \todo try to make everything go through this by putting conditions based on movement mode?
    if (withinJointAngleLimits(angle)) {
        desiredAngle = angle;
        return true;
    }
    else {
        return false;
    }
}

float RobotMotor::getDesiredAngle(void) {
    return desiredAngle;
}

int RobotMotor::calcDirection(float error) {
    if (error >= 0) {
        rotationDirection = directionModifier * COUNTER_CLOCKWISE;
    }
    else {
        rotationDirection = directionModifier * CLOCKWISE;
    }
    Serial.println(rotationDirection);
    return rotationDirection;
}

int RobotMotor::calcDirection(bool dir) {
    if (dir) {
        rotationDirection = directionModifier * CLOCKWISE;
    } else {
        rotationDirection = directionModifier * COUNTER_CLOCKWISE;
    }
    Serial.println(rotationDirection);
    return rotationDirection;
}

void RobotMotor::switchDirectionLogic(void) {
    directionModifier = directionModifier * -1;
}

int RobotMotor::getDirectionLogic(void) {
    return directionModifier;
}

void RobotMotor::setGearRatio(float ratio) {
    gearRatio = ratio;
    gearRatioReciprocal = 1 / ratio;
}

void RobotMotor::setOpenLoopGain(float loopGain) {
    openLoopGain = loopGain;
}

void RobotMotor::setMotorSpeed(float motorSpeed) {
    openLoopSpeed = motorSpeed;
    pidController.setOutputLimits(-motorSpeed, motorSpeed);
}

float RobotMotor::getMotorSpeed(void) {
    if (isOpenLoop){
        return openLoopSpeed;
    }
    else {
        return pidController.getMaxOutputValue();
    }
}

//! \todo needs to be tested to make sure that a call to calcCurrentAngle will return the same thing as getCurrentAngle
void RobotMotor::setSoftwareAngle(float angle) {
    if (isOpenLoop) {
        imaginedAngle = angle;
    }
    else {
        currentAngle = angle;
        encoderCount = directionModifier * angle * gearRatio * encoderResolution / 360.0; // does adding directionModifier fix it?
    }
}

float RobotMotor::getSoftwareAngle(void) {
    if (isOpenLoop) {
        return imaginedAngle;
    }
    else {
        return currentAngle;
    }
}

void RobotMotor::checkForActualPress(void) {
    if (sinceTrigger >= TRIGGER_DELAY) {
        // if the last interrupt was a press (meaning it's stabilized and in contact)
        // then there's a real press
        if (triggerState != 0) {
            actualPress = true;
            limitSwitchState = triggerState;
        }
        // otherwise it's not a real press
        // so the limit switch state should stay whatever it used to be
        // and so should actualPress
        else {
            ;
        }
        // either way, we should reset the triggered bool in wait for the next trigger
        triggered = false;
    }
}

void RobotMotor::goToSafeAngle(void) {
    stopRotation(); // stop turning of course
    // check which switch was hit, update the current angle as the max/min hardware angle,
    // then move the angle back to the max/min software angle
    if (limitSwitchState == COUNTER_CLOCKWISE) {
        setSoftwareAngle(maxHardAngle);
        forceToAngle(maxSoftAngle);
    }
    if (limitSwitchState == CLOCKWISE) {
        setSoftwareAngle(minHardAngle);
        forceToAngle(minSoftAngle);
    }
    // now that the behaviour is complete we can reset these,
    // in wait for the next trigger to be confirmed
    actualPress = false;
    limitSwitchState = 0;
}

void RobotMotor::homeMotor(char homingDir) {
    homingPass++;
    if (homingDir == 'i') { //(neg, cw)
        // set homing direction inwards
        forceToAngle(1.5 * minHardAngle);
    }
    else if (homingDir == 'o') { //(pos, ccw)
        forceToAngle(5 * maxHardAngle);
    }
    homingDone = false;
}

void RobotMotor::stopHoming(void) {
    homingDone = true;
    homingPass = 0;
    atSafeAngle = true;
    startedZeroing = false;
}

#endif
