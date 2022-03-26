#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "../../internal_comms/include/LSSServoMotor.h"
#include "RobotMotor.h"

class ServoMotor: public RobotMotor {
    public:
        static int numServoMotors;
        unsigned int servoId;

        ServoMotor(float gearRatio, int motorNum);
        void motorTimerInterrupt(void);
        /* movement helper functions */
        bool calcTurningDuration(float angle); //!< guesstimates how long to turn at the preset open loop motor speed to get to the desired position
        bool calcCurrentAngle(void);
        /* movement functions */
        void stopRotation(void);
        void setVelocity(int motorDir, float motorSpeed); //!< currently this actually activates the servo and makes it turn at a set speed/direction
        void goToCommandedAngle(void);
        void forceToAngle(float angle);
        void budge(bool dir);

        // stuff for open loop control
        unsigned int numMillis; //!< how many milliseconds for servo to reach desired position
        elapsedMillis timeCount; //!< how long has the servo been turning for

    private:
        LSSServoMotor servo = LSSServoMotor(&Serial5);
};

int ServoMotor::numServoMotors = 0; // must initialize variable outside of class

ServoMotor::ServoMotor(float gearRatio, int motorNum)
{
    numServoMotors++;
    this -> servoId = motorNum;
    // variables declared in RobotMotor require the this-> operator
    setGearRatio(gearRatio);
    this -> motorType = CONTINUOUS_SERVO;
    hasEncoder = false;
}

void ServoMotor::motorTimerInterrupt(void) {
    if (isBudging) {
        if (sinceBudgeCommand < BUDGE_TIMEOUT) {
            calcCurrentAngle();
            setVelocity(rotationDirection, openLoopSpeed);
        }
        else {
            isBudging = false;
            if (!atSafeAngle) {
                atSafeAngle = true; // alert homing stuff that it can go to next part
            }
            movementDone = true;
            stopRotation();
        }
    }
    else if (isOpenLoop) {
        // open loop control
        if (!movementDone && timeCount < numMillis) {
            calcCurrentAngle();
            setVelocity(rotationDirection, openLoopSpeed);
        }
        else {
            // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
            if (!atSafeAngle) {
                atSafeAngle = true; // alert homing stuff that it can go to next part
            }
            movementDone = true;
//            stopRotation();
        }
    }
    else if (!isOpenLoop) {
        if (!movementDone) {
            calcCurrentAngle();
            float output = pidController.updatePID(getSoftwareAngle(), getDesiredAngle());
            if (output == 0) {
                if (!atSafeAngle) {
                    atSafeAngle = true; // alert homing stuff that it can go to next part
                }
                movementDone = true;
                stopRotation();
            }
            else {
                int dir = calcDirection(output);
                setVelocity(dir, output);
            }
        }
        else {
            stopRotation();
            if (!atSafeAngle) {
                atSafeAngle = true; // alert homing stuff that it can go to next part
            }
        }
    }
}

bool ServoMotor::calcTurningDuration(float angle) {
    // if the error is big enough to justify movement
    // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
    if (fabs(angle) > pidController.getJointAngleTolerance()) {
        numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
        // Serial.println(numMillis);
        return true;
    }
    else {
        return false;
    }
}

bool ServoMotor::calcCurrentAngle(void) {
    if (isBudging) {
        imaginedAngle = startAngle + (float)rotationDirection * (float)sinceBudgeCommand * openLoopSpeed * gearRatioReciprocal / (1000.0 * openLoopGain);
        return true;
    }
    else if (isOpenLoop) {
        if (movementDone) {
            // imaginedAngle hasn't changed since motor hasn't moved and encoder isn't working
        }
        else if (timeCount < numMillis) {
            // if the motor is moving, calculate the angle based on how long it's been turning for
            imaginedAngle = startAngle + (desiredAngle - startAngle) * ((float)timeCount / (float)numMillis);
        }
        return true;
    }
    else if (hasEncoder) {
        currentAngle = directionModifier * (float) encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
        imaginedAngle = currentAngle;
        return true;
    }
    else {
        return false;
    }
}

void ServoMotor::stopRotation(void) {
    servo.writeServoCommand(servoId, "H");
    movementDone = true;
    isBudging = false;
}

// takes a direction and offset from SERVO_STOP and sends appropriate pwm signal to servo
void ServoMotor::setVelocity(int motorDir, float motorSpeed) {
//    if (!isOpenLoop) {
//        motorSpeed = fabs(motorSpeed);
//    }
//    // makes sure the speed is within the limits set in the pid during setup
//    if (motorSpeed * motorDir > pidController.getMaxOutputValue()) {
//        motorSpeed = pidController.getMaxOutputValue();
//    }
//    if (motorSpeed * motorDir < pidController.getMinOutputValue()) {
//        motorSpeed = pidController.getMinOutputValue();
//    }
//
//    // pulse time varies from 1000 to 2000, 1500 being the midpoint, so 500 is the offset from 1500
//    int pulseTime = SERVO_STOP + (motorSpeed * motorDir * 500 / 100);
//    if (pulseTime > 2000) {
//        pulseTime = 1000;
//    }
//    if (pulseTime < 1000) {
//        pulseTime = -1000;
//    }

    servo.writeServoCommand(servoId, "MD", motorDir * 100);
}

void ServoMotor::goToCommandedAngle(void) {
    if (isOpenLoop) {
        calcCurrentAngle();
        startAngle = getSoftwareAngle();
        openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
        calcDirection(openLoopError);
        if (calcTurningDuration(openLoopError)) {
            timeCount = 0;
            movementDone = false;
        }
        else {
        }
    }
    else {
        if (!isOpenLoop) {
            movementDone = false;
        }
    }
}

void ServoMotor::forceToAngle(float angle) {
    desiredAngle = angle;
    if (isOpenLoop) {
        calcCurrentAngle();
        startAngle = getSoftwareAngle();
        openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
        calcDirection(openLoopError);
        // calculates how many steps to take to get to the desired position, assuming no slipping
        numMillis = (fabs(openLoopError) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain;
        timeCount = 0;
        movementDone = false;
    }
    else if (!isOpenLoop) {
        movementDone = false;
    }
}

void ServoMotor::budge(bool dir) {
    calcCurrentAngle();
    float ang = getSoftwareAngle();
    bool canMove = true;
    if (hasAngleLimits) {
        if ( ( (dir) && (ang > maxJointAngle) ) || ( (!dir) && (ang < minJointAngle) ) ) {
            canMove = false;
        }
    }
    if (canMove) {
        calcDirection(dir);
        isBudging = true;
        movementDone = false;
        sinceBudgeCommand = 0;
        startAngle = getSoftwareAngle();
    }
}

#endif
