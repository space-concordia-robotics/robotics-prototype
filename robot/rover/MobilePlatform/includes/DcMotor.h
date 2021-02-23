#include "RobotMotor.h"
class DcMotor: public RobotMotor {
public:
    DcMotor(int dirPin, int pwmPin, float gearRatio, String motorName);
    void setVelocity(int motorDir, float dV, volatile float currentVelocity); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    int directionPin;
    int pwmPin;
    void calcCurrentVelocity(void);
    float getCurrentVelocity(void);
    float desiredVelocity;
    int desiredDirection;
    
    static void velocityHandler(DcMotor* motors, float throttle, float steering);
protected:
    // the following variables are specific to encoders
    int encoderResolution; // ticks per revolution
    volatile float currentVelocity; // can be updated within timer interrupts
//    float desiredVelocity;
//    int directionModifier;
//    float setDesiredVelocity(float desiredVelocity);

};
