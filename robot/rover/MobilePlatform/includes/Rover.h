#ifndef ROVER_H
#define ROVER_H
#include "DcMotor.h"
#include "Config.h"


const float width = 0.83;
const float length = 1.2;

const float radius = 0.14;
const float piRad  = 0.10472;
const float wheelBase = 0.33;

#define V_SENSE_PIN 39 // for reading battery voltage

/***** MOTORS *****/

#define ROVER_MOVE_TIMEOUT 500

#define ACCELERATION_RATE 5

typedef struct {
    float right_linear_velocity;
    float left_linear_velocity;
    float linear_velocity;
    float rotational_velocity;
    int16_t max_output_signal;
    int16_t min_output_signal;
} RoverState;

enum ServoNames{
    CENTER_FRONT_1_SERVO = 0,
    CENTER_FRONT_2_SERVO= 1,
    CENTER_BACK_1_SERVO = 2,
    CENTER_BACK_2_SERVO = 3
};
namespace Rover {

    extern RoverState roverState;

    extern SystemState systemStatus;

    void calculateRoverVelocity();

    void moveWheel(const MotorNames &,const uint8_t &, const int8_t& );

    void moveRover(const float & ,const float &);


    void attachServo(const ServoNames&, const uint8_t&);

    void moveServo(const ServoNames&, const uint8_t&);

    void stopMotors();

    void updateWheelVelocities();

    void decelerateRover();
}
#endif