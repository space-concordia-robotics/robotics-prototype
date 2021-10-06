#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include "RobotMotor.h"

class DcMotor: public RobotMotor {
    public:
        static int numDcMotors;

        DcMotor(int dirPin, int pwmPin, float gearRatio);
        void motorTimerInterrupt(void);
        /* movement helper functions */
        bool calcTurningDuration(float angle); //!< guesstimates how long to turn at the preset open loop motor speed to get to the desired position
        bool calcCurrentAngle(void);
        /* movement functions */
        void stopRotation(void);
        void setVelocity(int motorDir, float motorSpeed); //!< currently this actually activates the dc motor and makes it turn at a set speed/direction
        void goToCommandedAngle(void);
        void forceToAngle(float angle);
        void budge(bool dir);

        // stuff for open loop control
        unsigned int numMillis; //!< how many milliseconds for dc motor to reach desired position
        elapsedMillis timeCount; //!< how long has the dc motor been turning for

    private:
        int directionPin;
        int pwmPin;
};

int DcMotor::numDcMotors = 0; // must initialize variable outside of class

DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio):// if no encoder
    directionPin(dirPin), pwmPin(pwmPin)
{
    numDcMotors++;
    // variables declared in RobotMotor require the this-> operator
    setGearRatio(gearRatio);
    motorType = DC_MOTOR;
    hasEncoder = false;
}

void DcMotor::motorTimerInterrupt(void) {
    if (isBudging) {
        if (sinceBudgeCommand < BUDGE_TIMEOUT) {
            //Serial.write(rotationDirection);
            calcCurrentAngle();
            setVelocity(rotationDirection, openLoopSpeed);
        }
        else {
            isBudging = false;
            movementDone = true;
            stopRotation();
        }
    }
    // movementDone can be set elsewhere... so can numMillis, openLoopSpeed and rotationDirection (in open loop control)
    else if (isOpenLoop) { // open loop control
        if (!movementDone && timeCount <= numMillis) {
            // calculates the pwm to send to the motor and makes it move
            calcCurrentAngle();
            setVelocity(rotationDirection, openLoopSpeed);
        }
        else {
            if (!atSafeAngle) {
                atSafeAngle = true; // alert homing stuff that it can go to next part
            }
            movementDone = true;
            stopRotation();
        }
    }
    // would be nice to have some kind of check for the above functions so the command only runs if there's been a change
    // e.g. movementDone changed or the speed or numMillis changed
    else if (!isOpenLoop) {
        if (!movementDone) {
            calcCurrentAngle();
            // determine the speed of the motor until the next interrupt
            float output = pidController.updatePID(getSoftwareAngle(), getDesiredAngle());
            if (output == 0) {
                movementDone = true;
                stopRotation();
                if (!atSafeAngle) {
                    atSafeAngle = true; // alert homing stuff that it can go to next part
                }
            }
            else {
                int dir = calcDirection(output);
                // calculates the pwm to send to the motor and makes it move
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

bool DcMotor::calcTurningDuration(float angle) {
    // if the error is big enough to justify movement
    if (fabs(angle) > pidController.getJointAngleTolerance()) {
        // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
        numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
        return true;
    }
    else {
        return false;
    }
}

bool DcMotor::calcCurrentAngle(void) {
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
    else if (hasEncoder) { // closed loop and has encoder
        currentAngle = directionModifier * (float)encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
        imaginedAngle = currentAngle;
        return true;
    }
    else { // closed loop and has no encoder
        return false;
    }
}

void DcMotor::stopRotation(void) {
    analogWrite(pwmPin, 0);
    movementDone = true;
    isBudging = false;
}

void DcMotor::setVelocity(int motorDir, float motorSpeed) {
    if (!isOpenLoop) {
        motorSpeed = fabs(motorSpeed);
    }
    // makes sure the speed is within the limits set in the pid during setup
    if (motorSpeed * motorDir > pidController.getMaxOutputValue()) {
        motorSpeed = pidController.getMaxOutputValue();
    }
    if (motorSpeed * motorDir < pidController.getMinOutputValue()) {
        motorSpeed = pidController.getMinOutputValue();
    }

    switch (motorDir) {
        case CLOCKWISE: {
            //PORTD |= (0 << 3);
            digitalWrite(directionPin, LOW);
            //Serial.print(motorDir);
            break;
        }

        case COUNTER_CLOCKWISE: {
            //PORTD |= (1 << 3);
            pinMode(directionPin, OUTPUT);
            digitalWrite(directionPin, HIGH);
            //Serial.print(motorDir);
            break;
        }
    }

    int dutyCycle = motorSpeed * 255 / 100;
    analogWrite(pwmPin, dutyCycle);
}

void DcMotor::goToCommandedAngle(void) {
    if (isOpenLoop) {
        calcCurrentAngle();
        startAngle = getSoftwareAngle();
        openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
        calcDirection(openLoopError); // determine rotation direction and save the value
        // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
        if (calcTurningDuration(openLoopError)) { // returns false if the open loop error is too small
            timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
            movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
        }
        else {
        }
    }
    else if (!isOpenLoop) {
        // all the heavy lifting for closed loop control is done in the timer interrupt
        movementDone = false;
    }
}

void DcMotor::forceToAngle(float angle) {
    desiredAngle = angle;
    if (isOpenLoop) {
        calcCurrentAngle();
        startAngle = getSoftwareAngle();
        openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
        calcDirection(openLoopError);
        // calculates how many milliseconds it will take to get to the desired position, assuming no slipping
        numMillis = (fabs(openLoopError) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain;
        timeCount = 0;
        movementDone = false;
    }
    else if (!isOpenLoop) {
        movementDone = false;
    }
}

void DcMotor::budge(bool dir) {
    calcCurrentAngle();
    float ang = getSoftwareAngle();
    bool canMove = true;
    if (hasAngleLimits) {
        if ((dir && (ang > maxJointAngle)) || (!dir && (ang < minJointAngle))) {
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
