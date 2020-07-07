#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include "../PinSetup/PinSetup.h"
#include "../RobotMotor/RobotMotor.h"

#define MIN_STEP_INTERVAL 3000
#define MAX_STEP_INTERVAL 50000

// multipliers for how bit of an angle the stepper moves each step
#define FULL_STEP 1
#define HALF_STEP 0.5
#define QUARTER_STEP 0.25
#define EIGHTH_STEP 0.125
#define SIXTEENTH_STEP 0.0625

class StepperMotor: public RobotMotor {
  public:
    static int numStepperMotors;
    float stepResolution; //!< the smallest angle increment attainable by the shaft once the stepping mode is known

    StepperMotor(int enablePin, int dirPin, int stepPin, float stepRes, float stepMode, float gearRatio);
    void motorTimerInterrupt(IntervalTimer & timer);
    /* movement helper functions */
    bool calcNumSteps(float angle); //!< calculates how many steps to take to get to the desired position, assuming no slipping
    bool calcCurrentAngle(void);
    void enablePower(void);
    void disablePower(void);
    /* movement functions */
    void stopRotation(void);
    void singleStep();
    void setVelocity(int motorDir, float motorSpeed);
    void goToCommandedAngle(void);
    void forceToAngle(float angle);
    void budge(int dir);

    // stuff for open loop control
    int numSteps; //!< how many steps to take for stepper to reach desired position
    volatile int stepCount; //!< how many steps the stepper has taken since it started moving
    volatile int nextInterval; //!< how long to wait until the next step

  private:
    int enablePin, directionPin, stepPin;
    float fullStepResolution, steppingMode;
};

#endif
