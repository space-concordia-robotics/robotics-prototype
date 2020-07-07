#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include "../PinSetup/PinSetup.h"
#include "../RobotMotor/RobotMotor.h"

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
    void budge(int dir);

    // stuff for open loop control
    unsigned int numMillis; //!< how many milliseconds for dc motor to reach desired position
    elapsedMillis timeCount; //!< how long has the dc motor been turning for

  private:
    int directionPin;
    int pwmPin;
};


#endif
