#ifndef CORE_H
#define CORE_H
#include "Arduino.h"
#include <Servo.h>
#include <SoftwareSerial.h>

/* comms */
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 20
#define FEEDBACK_PRINT_INTERVAL 50
#define LED_BLINK_INTERVAL 1000
#define SENSOR_READ_INTERVAL 200
#define SENSOR_TIMEOUT 20
#define THROTTLE_TIMEOUT 200
#define MOTOR_CONTROL_INTERVAL 10

#define SERVO_STOP 93
#define FRONT_BASE_DEFAULT_PWM 65
#define REAR_BASE_DEFAULT_PWM 35

/* more variables */
#define MAX_INPUT_VALUE 49  // maximum speed signal from controller
#define MIN_INPUT_VALUE -MAX_INPUT_VALUE // minimum speed signal from controller
#define MAX_PWM_VALUE 255
#define MIN_PWM_VALUE -MAX_PWM_VALUE
#define MAX_RPM_VALUE MAX_RPM
#define MIN_RPM_VALUE -MAX_RPM


extern int loop_state; 
extern int button;
extern int i; // Input values for set velocity functions
extern float throttle;
extern float steering;  
extern float heading ; // Input values for set velocity functions
extern float maxOutputSignal;
extern float minOutputSignal;
extern int leftMotorDirection; // CCW =1 or CW =-1
extern int rightMotorDirection; // CCW =1 or CW =-1
extern float desiredVelocityRight;
extern float  desiredVelocityLeft;
extern String cmd;
extern int motorNumber;

#endif
