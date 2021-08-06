#include <cstdio>
#include "PidController.h"
#define PULSES_PER_REV     14
#define GEAR_RATIO         188.61
#define MAX_RPM            30


#define MAX_INPUT_VALUE 49  // maximum speed signal from controller
#define MIN_INPUT_VALUE -MAX_INPUT_VALUE // minimum speed signal from controller
#define MAX_PWM_VALUE 255
#define MIN_PWM_VALUE -MAX_PWM_VALUE
#define MAX_RPM_VALUE MAX_RPM
#define MIN_RPM_VALUE -MAX_RPM

enum MotorNames{
    LEFT_FRONT=0,
    LEFT_MIDDLE=1,
    LEFT_BACK=2,
    RIGHT_FRONT=3,
    RIGHT_MIDDLE=4,
    RIGHT_BACK=5
};
enum motor_direction { CW = -1, CCW = 1 };

class InterruptHandler{
public:
    static volatile u_int32_t LEFT_FRONT_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t LEFT_FRONT_MOTOR_DT;
    static volatile u_int32_t LEFT_FRONT_MOTOR_PREV_DT;

    static volatile u_int32_t LEFT_MIDDLE_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t LEFT_MIDDLE_MOTOR_DT;
    static volatile u_int32_t LEFT_MIDDLE_MOTOR_PREV_DT;

    static volatile u_int32_t LEFT_BACK_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t LEFT_BACK_MOTOR_DT;
    static volatile u_int32_t LEFT_BACK_MOTOR_PREV_DT;

    static volatile u_int32_t RIGHT_FRONT_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t RIGHT_FRONT_MOTOR_DT;
    static volatile u_int32_t RIGHT_FRONT_MOTOR_PREV_DT;

    static volatile u_int32_t RIGHT_MIDDLE_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t RIGHT_MIDDLE_MOTOR_DT;
    static volatile u_int32_t RIGHT_MIDDLE_MOTOR_PREV_DT;

    static volatile u_int32_t RIGHT_BACK_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t RIGHT_BACK_MOTOR_DT;
    static volatile u_int32_t RIGHT_BACK_MOTOR_PREV_DT;

    static void RightFrontMotorInterruptHandler(){
        RIGHT_FRONT_MOTOR_ENCODER_COUNT++;
        RIGHT_FRONT_MOTOR_DT = micros() - RIGHT_FRONT_MOTOR_PREV_DT;
        RIGHT_FRONT_MOTOR_PREV_DT = micros();
    }
    static void RightMiddleMotorInterruptHandler(){
        RIGHT_MIDDLE_MOTOR_ENCODER_COUNT++;
        RIGHT_MIDDLE_MOTOR_DT = micros() - RIGHT_MIDDLE_MOTOR_PREV_DT;
        RIGHT_MIDDLE_MOTOR_PREV_DT = micros();
    }
    static void RightBackMotorInterruptHandler(){
        RIGHT_BACK_MOTOR_ENCODER_COUNT++;
        RIGHT_BACK_MOTOR_DT = micros() - RIGHT_BACK_MOTOR_PREV_DT;
        RIGHT_BACK_MOTOR_PREV_DT = micros();
    }
    static void LeftFrontMotorInterruptHandler(){
        LEFT_FRONT_MOTOR_ENCODER_COUNT++;
        LEFT_FRONT_MOTOR_DT = micros() - LEFT_FRONT_MOTOR_PREV_DT;
        LEFT_FRONT_MOTOR_PREV_DT = micros();
    }
    static void LeftMiddleMotorInterruptHandler(){
        LEFT_MIDDLE_MOTOR_ENCODER_COUNT++;
        LEFT_MIDDLE_MOTOR_DT = micros() - LEFT_MIDDLE_MOTOR_PREV_DT;
        LEFT_MIDDLE_MOTOR_PREV_DT = micros();
    }
    static void LeftBackMotorInterruptHandler(){
        LEFT_BACK_MOTOR_ENCODER_COUNT++;
        LEFT_BACK_MOTOR_DT = micros() - LEFT_BACK_MOTOR_PREV_DT;
        LEFT_BACK_MOTOR_PREV_DT = micros();
    }

    static uint32_t getEncoderCount(MotorNames name){
        switch (name) {
            case RIGHT_FRONT:
                return RIGHT_FRONT_MOTOR_ENCODER_COUNT;
            case RIGHT_MIDDLE:
                return RIGHT_MIDDLE_MOTOR_ENCODER_COUNT;
            case RIGHT_BACK:
                return RIGHT_BACK_MOTOR_ENCODER_COUNT;
            case LEFT_FRONT:
                return LEFT_FRONT_MOTOR_ENCODER_COUNT;
            case LEFT_MIDDLE:
                return LEFT_MIDDLE_MOTOR_ENCODER_COUNT;
            case LEFT_BACK:
                return LEFT_BACK_MOTOR_ENCODER_COUNT;

        }
    }
    static uint32_t getMotorDt(MotorNames name){
        switch (name) {
            case RIGHT_FRONT:
                return RIGHT_FRONT_MOTOR_DT;
            case RIGHT_MIDDLE:
                return RIGHT_MIDDLE_MOTOR_DT;
            case RIGHT_BACK:
                return RIGHT_BACK_MOTOR_DT;
            case LEFT_FRONT:
                return LEFT_FRONT_MOTOR_DT;
            case LEFT_MIDDLE:
                return LEFT_MIDDLE_MOTOR_DT;
            case LEFT_BACK:
                return LEFT_BACK_MOTOR_DT;

        }
    }
    static void reset(MotorNames name) {
        switch (name) {
            case RIGHT_FRONT: {
                RIGHT_FRONT_MOTOR_DT = 0;
                RIGHT_FRONT_MOTOR_ENCODER_COUNT = 0;
                RIGHT_FRONT_MOTOR_PREV_DT = 0;
                break;
            }
            case RIGHT_MIDDLE: {
                RIGHT_MIDDLE_MOTOR_DT = 0;
                RIGHT_MIDDLE_MOTOR_ENCODER_COUNT = 0;
                RIGHT_MIDDLE_MOTOR_PREV_DT = 0;
                break;
            }
            case RIGHT_BACK: {
                RIGHT_BACK_MOTOR_DT = 0;
                RIGHT_BACK_MOTOR_ENCODER_COUNT = 0;
                RIGHT_BACK_MOTOR_PREV_DT=0;
            } break;

            case LEFT_FRONT: {
                InterruptHandler::LEFT_FRONT_MOTOR_DT = 0;
                InterruptHandler::LEFT_FRONT_MOTOR_ENCODER_COUNT = 0;
                InterruptHandler::LEFT_FRONT_MOTOR_PREV_DT = 0;
                break;
            }
            case LEFT_MIDDLE:{
                LEFT_MIDDLE_MOTOR_ENCODER_COUNT = 0;
                LEFT_MIDDLE_MOTOR_DT = 0;
                LEFT_MIDDLE_MOTOR_PREV_DT = 0;
                break;
            }

            case LEFT_BACK: {
                LEFT_BACK_MOTOR_ENCODER_COUNT = 0;
                LEFT_BACK_MOTOR_DT = 0;
                LEFT_BACK_MOTOR_PREV_DT = 0;
                break;
            }
        }
    }
};

class DcMotor {

    bool isEnabled;
    MotorNames name;
    PidController* pidController;
    bool isOpenLoop;
    uint16_t maxOutputSignal;
    uint16_t minOutputSignal;
    float gearRatio;
    float gearRatioReciprocal; // preemptively reduce floating point calculation time

    volatile float currentVelocity;
    uint8_t encoderPinA;
    uint8_t encoderPinB;
    float encoderResolution;
    float encoderResolutionReciprocal;
    bool isEncoderEnabled;
    motor_direction desiredDirection;
    int16_t desiredVelocity;
    uint8_t dirPin;
    uint8_t pwmPin;
public:
    void closeLoop();
    void openLoop();
    PidController* getPidController(void ) const;
    void updateDesiredVelocity(motor_direction,int16_t);
    void attachEncoder(int encA, int encB, float encRes,void (*)(void));
    DcMotor(MotorNames, uint8_t , uint8_t , float );
    void setVelocity(); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    void setEnabled(bool isEnabled);
    void initPidController(float,float,float);

    motor_direction getDesiredDirection() const;

    int16_t getDesiredVelocity() const;
    float getCurrentVelocity() const;
    void calcCurrentVelocity();
    DcMotor();
};
