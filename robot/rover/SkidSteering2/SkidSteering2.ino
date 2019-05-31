#include "pins.h"
#include "motors.h"
#include <Servo.h>

/* comms */
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 20
#define FEEDBACK_PRINT_INTERVAL 1000//50
#define LED_BLINK_INTERVAL 1000
#define SENSOR_READ_INTERVAL 1000//200
#define SENSOR_TIMEOUT 20

/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
#define DEVEL_MODE_1 1
//#define DEVEL_MODE_2 2

#if defined(DEVEL_MODE_1)
// serial communication over usb
#define UART_PORT Serial
#elif defined(DEVEL_MODE_2)
// serial communication over uart with odroid, teensy plugged into pcb and odroid
#define UART_PORT Serial1
#endif

/* more variables */
#define MAX_INPUT_VALUE 49  // maximum speed signal from controller
#define MIN_INPUT_VALUE -MAX_INPUT_VALUE // minimum speed signal from controller
#define MAX_PWM_VALUE 255
#define MIN_PWM_VALUE -MAX_PWM_VALUE
#define MAX_RPM_VALUE MAX_RPM
#define MIN_RPM_VALUE -MAX_RPM

elapsedMillis sinceFeedbackPrint; // timer for sending motor speeds and battery measurements
elapsedMillis sinceLedToggle; // timer for heartbeat
elapsedMillis sinceSensorRead; // timer for reading battery, gps and imu data
String cmd;

float maxOutputSignal, minOutputSignal;
int motorNumber;
int leftMotorDirection; // CCW =1 or CW =-1
int rightMotorDirection; // CCW =1 or CW =-1
float desiredVelocityRight = 0;
float desiredVelocityLeft = 0;
float d = 0.33; //distance between left and right wheels
float radius = 14;
float forwardVelocity, rotationalVelocity, rightLinearVelocity, leftLinearVelocity;
String rotation; // Rotation direction of the whole rover
float throttle = 0, steering = 0, heading = 0; // Input values for set velocity functions
int loop_state, button, i; // Input values for set velocity functions

// constructors
DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO, "Front Right Motor");
DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO, "Middle Right Motor");
DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO, "Rear Right Motor");
DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO, "Front Left Motor");
DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO, "Middle Left Motor");
DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO, "Rear Left Motor");

DcMotor motorList[] = {RF, RM, RB, LF, LM, LB};

Servo frontSide, frontBase, topSide, topBase;
Servo servoList[] = {frontSide, frontBase, topSide, topBase};

/* function declarations */
// initializers
void attachServos(void); // attach pins to servo objects
void initEncoders(void); // Encoder initiation (attach interrups and pinModes for wheel encoders
void initPids(void); // Initiate PID for DMotor
// during operation
void velocityHandler(float throttle, float steering);
void roverVelocityCalculator(void);
void motor_encoder_interrupt(int motorNumber);

// these includes must be placed exactly where and how they are in the code
#include "helpers.h"
#include "Vsense.h"
#include "commands.h"
Commands Cmds;
#include "navigation.h"

// encoder interrupts
void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);

void setup() {
  // initialize serial communications at 115200 bps:
  UART_PORT.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
  UART_PORT.setTimeout(SERIAL_TIMEOUT);

  //ser_flush();
  initPins();
  initEncoders();
  initPids();
  attachServos();
  initNav(Cmds);

  if (Cmds.isOpenLoop) {
    maxOutputSignal = MAX_PWM_VALUE;
    minOutputSignal = MIN_PWM_VALUE;
  }
  else {
    maxOutputSignal = MAX_RPM_VALUE;
    minOutputSignal = MIN_RPM_VALUE;
  }
  delay(200); // needs this to actually print the following stuff
  Cmds.setupMessage();
}

void loop() {
  // incoming format example: "5:7"
  // this represents the speed for throttle:steering
  // as well as direction by the positive/negative sign
  if (UART_PORT.available()) {
    cmd = UART_PORT.readStringUntil('\n');
    cmd.trim();
    //ser_flush();
    Cmds.handler(cmd, "Serial");
  }
  if (sinceSensorRead > SENSOR_READ_INTERVAL) {
    vbatt_read();
    navHandler(Cmds);
    sinceSensorRead = 0;
  }
  if (sinceLedToggle > LED_BLINK_INTERVAL) {
    toggleLed();
    sinceLedToggle = 0;
  }
  if (sinceFeedbackPrint > FEEDBACK_PRINT_INTERVAL) {
    UART_PORT.print("ASTRO Motor Speeds: ");
    if (Cmds.isEnc) {
      for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
        UART_PORT.print(motorList[i].getCurrentVelocity());
        if (i != RobotMotor::numMotors - 1) UART_PORT.print(", ");
      }
      UART_PORT.println("");
    }
    else {
      UART_PORT.print(String(desiredVelocityRight) + ", ");
      UART_PORT.print(String(desiredVelocityRight) + ", ");
      UART_PORT.print(String(desiredVelocityRight) + ", ");
      UART_PORT.print(String(desiredVelocityLeft) + ", ");
      UART_PORT.print(String(desiredVelocityLeft) + ", ");
      UART_PORT.print(String(desiredVelocityLeft));
      UART_PORT.println("");
    }
    sinceFeedbackPrint = 0;
  }
}

void velocityHandler(float throttle, float steering) {
  // steering of 0 means both motors at throttle.
  // as steering moves away from 0, one motor keeps throttle.
  // the other slows down or even reverses based on the steering value.
  // deg is mapped between -1 and 1 so 0 means no speed and the extremes mean full throttle in either direction.
  float multiplier;
  // If statement for CASE 1: steering toward the RIGHT
  if (steering > 0 ) {
    multiplier = mapFloat(steering, 0, MAX_INPUT_VALUE, 1, -1);
    desiredVelocityRight = mapFloat(throttle * multiplier, MIN_INPUT_VALUE, MAX_INPUT_VALUE, minOutputSignal, maxOutputSignal);
    desiredVelocityLeft = mapFloat(throttle, MIN_INPUT_VALUE, MAX_INPUT_VALUE,  minOutputSignal, maxOutputSignal);
    //rotation = "CW";
  }
  // If statement for CASE 2: steering toward the LEFT or not steering
  if (steering <= 0 ) {
    multiplier = mapFloat(steering, MIN_INPUT_VALUE, 0, -1, 1);
    desiredVelocityRight = mapFloat(throttle, MIN_INPUT_VALUE, MAX_INPUT_VALUE, minOutputSignal, maxOutputSignal);
    desiredVelocityLeft = mapFloat(throttle * multiplier, MIN_INPUT_VALUE, MAX_INPUT_VALUE, minOutputSignal, maxOutputSignal);
    //rotation = "CCW";
  }
  if (desiredVelocityLeft > 0 ) {
    leftMotorDirection = 1;
  }
  else {
    leftMotorDirection = -1;
  }

  if (desiredVelocityRight < 0 ) {
    rightMotorDirection = -1;
  }
  else {
    rightMotorDirection = 1;
  }

  RF.calcCurrentVelocity();
  RM.calcCurrentVelocity();
  RB.calcCurrentVelocity();
  LF.calcCurrentVelocity();
  LM.calcCurrentVelocity();
  LB.calcCurrentVelocity();

  RF.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RF.getCurrentVelocity());
  RM.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RM.getCurrentVelocity());
  RB.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RB.getCurrentVelocity());
  LF.setVelocity(rightMotorDirection, abs(desiredVelocityLeft), LF.getCurrentVelocity());
  LM.setVelocity(rightMotorDirection, abs(desiredVelocityLeft), LM.getCurrentVelocity());
  LB.setVelocity(rightMotorDirection, abs(desiredVelocityLeft), LB.getCurrentVelocity());
}

void roverVelocityCalculator(void) {
  rightLinearVelocity = (RF.getDirection() * RF.getCurrentVelocity() + RM.getDirection() * RM.getCurrentVelocity() + RB.getDirection() * RB.getCurrentVelocity()) * radius * 0.10472;
  leftLinearVelocity = (LF.getDirection() * LF.getCurrentVelocity() + LM.getDirection() * LM.getCurrentVelocity() + LB.getDirection() * LB.getCurrentVelocity()) * radius * 0.10472;

  forwardVelocity = (rightLinearVelocity + leftLinearVelocity)  / 6;
  rotationalVelocity = (leftLinearVelocity - rightLinearVelocity) / d;
}

//! attach the servos to pins
void attachServos() {
  frontSide.attach(FS_SERVO);
  frontBase.attach(FB_SERVO);
  topSide.attach(TS_SERVO);
  topBase.attach(TB_SERVO);
}
//! Initiate encoder for dcMotor objects and pinModes
void initEncoders(void) {
  RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
  pinMode(RF.encoderPinB, INPUT_PULLUP);
  pinMode(RF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
  pinMode(RM.encoderPinB, INPUT_PULLUP);
  pinMode(RM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
  pinMode(RB.encoderPinB, INPUT_PULLUP);
  pinMode(RB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
  pinMode(LF.encoderPinB, INPUT_PULLUP);
  pinMode(LF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
  pinMode(LM.encoderPinB, INPUT_PULLUP);
  pinMode(LM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
  pinMode(LB.encoderPinB, INPUT_PULLUP);
  pinMode(LB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);
}
//! Initiate PID objects for Dc Motors
void initPids(void) {
  /*
    RF.pidController.setJointVelocityTolerance(2.0 * RF.gearRatioReciprocal);
    RM.pidController.setJointVelocityTolerance(2.0 * RM.gearRatioReciprocal);
    RB.pidController.setJointVelocityTolerance(2.0 * RB.gearRatioReciprocal);

    LF.pidController.setJointVelocityTolerance(2.0 * LF.gearRatioReciprocal);
    LM.pidController.setJointVelocityTolerance(2.0 * LM.gearRatioReciprocal);
    LB.pidController.setJointVelocityTolerance(2.0 * LB.gearRatioReciprocal);

    RF.pidController.setOutputLimits(-50, 50, 5.0);
    RM.pidController.setOutputLimits(-50, 50, 5.0);
    RB.pidController.setOutputLimits(-50, 50, 5.0);

    LF.pidController.setOutputLimits(-50, 50, 5.0);
    LM.pidController.setOutputLimits(-50, 50, 5.0);
    LB.pidController.setOutputLimits(-50, 50, 5.0);
  */
}

void rf_encoder_interrupt(void) {
  RF.dt += micros() - RF.prevTime;
  RF.prevTime = micros();
  RF.encoderCount++;
  //    motorList[0].setVelocity(1 , 0, motorList[0].getCurrentVelocity());
}
void rm_encoder_interrupt(void) {
  RM.dt += micros() - RM.prevTime;
  RM.prevTime = micros();
  RM.encoderCount++;
}
void rb_encoder_interrupt(void) {
  RB.dt += micros() - RB.prevTime;
  RB.prevTime = micros();
  RB.encoderCount++;
}
void lf_encoder_interrupt(void) {
  LF.dt += micros() - LF.prevTime;
  LF.prevTime = micros();
  LF.encoderCount++;
}
void lm_encoder_interrupt(void) {
  LM.dt += micros() - LM.prevTime;
  LM.prevTime = micros();
  LM.encoderCount++;
}
void lb_encoder_interrupt(void) {
  LB.dt += micros() - LB.prevTime;
  LB.prevTime = micros();
  LB.encoderCount++;
}
