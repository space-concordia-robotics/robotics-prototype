#ifndef ROVER_H
#define ROVER_H
#include "DcMotor.h"
#include "Servo.h"

const float width = 0.83;
const float length = 1.2;

const float radius = 0.14;
const float piRad  = 0.10472;
const float wheelBase = 0.33;

#define V_SENSE_PIN 39 // for reading battery voltage

/***** MOTORS *****/

#define NUM_MOTORS 6
#define ROVER_MOVE_TIMEOUT 500
#define ACCELERATION_RATE 5


enum ServoNames{
    FRONT_SIDE_SERVO = 0,
    FRONT_BASE_SERVO = 1,
    REAR_SIDE_SERVO= 2,
    REAR_BASE_SERVO = 3
};

typedef struct {
    float right_linear_velocity;
    float left_linear_velocity;
    float linear_velocity;
    float rotational_velocity;
    int16_t max_output_signal;
    int16_t min_output_signal;
} RoverState;

namespace Rover {

    extern RoverState roverState;

    extern SystemState systemStatus;

    void calculateRoverVelocity();

    void attachServo(const ServoNames&, const uint8_t&);

    void writeToServo(const ServoNames&, const int16_t&);

    void moveWheel(const MotorNames &,uint8_t ,uint8_t);

    void moveRover(const float & ,const float &);

    void stopMotors();

    void updateWheelVelocities();

    void decelerateRover();
}
#endif