/* still in idea phase */
#define DEVEL_MODE_1 1 // sends messages with Serial, everything unlocked
//#define DEVEL_MODE_2 2 // sends messages with Serial1, everything unlocked
//#define DEBUG_MODE 3 // sends messages with ROSserial, everything unlocked
//#define USER_MODE 4 // sends messages with ROSserial, functionality restricted
// debug statements shouldn't be sent if ros is working

#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#define DEBUG_MAIN 10 // debug messages during main loop
//#define DEBUG_PARSING 11 // debug messages during parsing function
//#define DEBUG_VERIFYING 12 // debug messages during verification function
//#define DEBUG_ENCODERS 13 // debug messages during encoder interrupts
//#define DEBUG_PID 14 // debug messages during pid loop calculations
#define DEBUG_SWITCHES 15 // debug messages during limit switch interrupts
//#define DEBUG_DC_TIMER 16 // debug messages during dc timer interrupts
//#define DEBUG_SERVO_TIMER 17 // debug messages during servo timer interrupts
//#define DEBUG_STEPPER_3_TIMER 18 // debug messages during stepper 3 timer interrupts
//#define DEBUG_STEPPER_4_TIMER 19 // debug messages during stepper 3 timer interrupts
#endif

/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
// serial communication over usb with pc, teensy not connected to odroid

#if defined(DEVEL_MODE_1)
#define UART_PORT Serial
// serial communication over uart with odroid, teensy plugged into pcb and odroid
#elif defined(DEVEL_MODE_2)
#define UART_PORT Serial1
#endif
#if defined(DEBUG_MODE) || defined(USER_MODE)
#define USE_TEENSY_HW_SERIAL 0 // this will make ArduinoHardware.h use hardware serial instead of usb serial
#endif
/*
  choosing serial1 vs rosserial could be compile-time, since serial1 is only really useful
  for debugging and won't be used when the rover is in action. however, a runtime option
  could be useful as in both cases the teensy is communicating solely with the odroid.
  it might be desirable to switch between modes without recompiling.
  finally, unlocking extra options should be runtime as it should be easily accessible.
*/
// includes must come after the above UART_PORT definition as it's used in other files.
// perhaps it should be placed in pinsetup.h (which has to be renamed anyway)...
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "PinSetup.h"
#include "Parser.h"
// #include "Notes.h" // holds todo info
// #include "Ideas.h" // holds bits of code that haven't been implemented
#include "PidController.h"
#include "RobotMotor.h"
#include "StepperMotor.h"
#include "DcMotor.h"
#include "ServoMotor.h"
/* interrupt priorities */
// motor control interrupts main loop, encoders interrupt motor control, limit switches interrupt encoder calculations
#define LIMIT_SWITCH_NVIC_PRIORITY 100
#define ENCODER_NVIC_PRIORITY LIMIT_SWITCH_NVIC_PRIORITY + 4
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4
/* serial */
#define BAUD_RATE 115200 // serial bit rate
#define SERIAL_PRINT_INTERVAL 1000 // how often should teensy send angle data
#define SERIAL_READ_TIMEOUT 50 // how often should the serial port be read
#define BUFFER_SIZE 100 // size of the buffer for the serial commands
/* comms */
char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
String messageBar = "======================================================="; // for clarity of print statements
// kinda weird that this comes out of nowhere... maybe should be Parser.commandInfo or something. or define it here instead of parser
/*
  info from parsing functionality is packaged and given to motor control functionality.
  many of these are set to 0 so that the message can reset, thus making sure that
  the code later on doesn't inadvertently make a motor move when it wasn't supposed to
*/
commandInfo motorCommand, emptyMotorCommand; // emptyMotorCommand is used to reset the struct when the loop restarts
Parser Parser; // object which parses and verifies commands

bool msgReceived = false;
bool msgIsValid = false;

// develmode1 actually isn't for ros... i will have to change things if i want ros over usb
#ifdef DEVEL_MODE_1 // using the USB port
//ros::NodeHandle nh;
#elif defined(DEBUG_MODE) || defined(USER_MODE) // using hardware serial (Serial1 in this case)
// the following commented block allows you to choose the hardware serial port
/*
  class NewHardware : public ArduinoHardware {
  public:
  long baud = 57600;
  NewHardware():ArduinoHardware(&Serial1, baud){}; // place the serial port of your choosing (1 to 6)
  };

  ros::NodeHandle_<NewHardware> nh;
*/
// otherwise just use this
ros::NodeHandle nh;
#endif

void messageCallback(const std_msgs::String& cmd_message) {
  msgReceived = true;
  int i = 0;
  while (cmd_message.data[i] != '\0') {
    serialBuffer[i] = cmd_message.data[i];
  }
  Parser.parseCommand(motorCommand, serialBuffer);
  if (Parser.verifCommand(motorCommand)) {
    msgIsValid = true;
  }
  memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
}
ros::Subscriber<std_msgs::String> cmdSubscriber("arm_command", &messageCallback);

char m1FrameId[] = "/m1_angle";
char m2FrameId[] = "/m2_angle";
char m3FrameId[] = "/m3_angle";
char m4FrameId[] = "/m4_angle";
char m5FrameId[] = "/m5_angle";
char m6FrameId[] = "/m6_angle";
sensor_msgs::JointState m1_angle_msg;
sensor_msgs::JointState m2_angle_msg;
sensor_msgs::JointState m3_angle_msg;
sensor_msgs::JointState m4_angle_msg;
sensor_msgs::JointState m5_angle_msg;
sensor_msgs::JointState m6_angle_msg;
sensor_msgs::JointState angleMessages[NUM_MOTORS] = {m1_angle_msg, m2_angle_msg, m3_angle_msg, m4_angle_msg, m5_angle_msg, m6_angle_msg};
ros::Publisher pub_m1("m1_joint_state", &m1_angle_msg);
ros::Publisher pub_m2("m2_joint_state", &m2_angle_msg);
ros::Publisher pub_m3("m3_joint_state", &m3_angle_msg);
ros::Publisher pub_m4("m4_joint_state", &m4_angle_msg);
ros::Publisher pub_m5("m5_joint_state", &m5_angle_msg);
ros::Publisher pub_m6("m6_joint_state", &m6_angle_msg);

/* motors */
// quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states
const int encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// instantiate motor objects here:
DcMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO); // for cytron
// DcMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B); // sabertooth
DcMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO); // for cytron
StepperMotor motor3(M3_ENABLE_PIN, M3_DIR_PIN, M3_STEP_PIN, M3_STEP_RESOLUTION, FULL_STEP, M3_GEAR_RATIO);
StepperMotor motor4(M4_ENABLE_PIN, M4_DIR_PIN, M4_STEP_PIN, M4_STEP_RESOLUTION, FULL_STEP, M4_GEAR_RATIO);
ServoMotor motor5(M5_PWM_PIN, M5_GEAR_RATIO);
ServoMotor motor6(M6_PWM_PIN, M6_GEAR_RATIO);
// motor array prep work: making pointers to motor objects
DcMotor * m1 = & motor1;
DcMotor * m2 = & motor2;
StepperMotor * m3 = & motor3;
StepperMotor * m4 = & motor4;
ServoMotor * m5 = & motor5;
ServoMotor * m6 = & motor6;
// I can use this instead of switch/case statements by doing motorArray[motornumber]->attribute
RobotMotor * motorArray[] = { m1, m2, m3, m4, m5, m6 };

// instantiate timers here:
IntervalTimer dcTimer; // motors 1&2
IntervalTimer m3StepperTimer;
IntervalTimer m4StepperTimer;
IntervalTimer servoTimer; // motors 5&6
// these are a nicer way of timing events than using millis()
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent
elapsedMillis sinceStepperCheck; // how long since last time stepper angle was verified
/* function declarations */
void printMotorAngles(void); // sends all motor angles over serial
// all interrupt service routines (ISRs) must be global functions to work
// declare encoder interrupt service routines

#ifdef M1_ENCODER_PORT
void m1_encoder_interrupt(void);
#endif
#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void);
#endif
#ifdef M3_ENCODER_PORT
void m3_encoder_interrupt(void);
#endif
#ifdef M4_ENCODER_PORT
void m4_encoder_interrupt(void);
#endif
#ifdef M5_ENCODER_PORT
void m5_encoder_interrupt(void);
#endif
#ifdef M6_ENCODER_PORT
void m6_encoder_interrupt(void);
#endif

// declare limit switch interrupt service routines
void m1CwISR(void);
void m1CcwISR(void);
void m2FlexISR(void);
void m2ExtendISR(void);
void m3FlexISR(void);
void m3ExtendISR(void);
void m4FlexISR(void);
void m4ExtendISR(void);
//void m5CwISR(void);
//void m5CcwISR(void);
//void m6ExtendISR(void);
// declare timer interrupt service routines, where the motors actually get controlled
void dcInterrupt(void); // manages motors 1&2
void servoInterrupt(void); // manages motors 5&6
// stepper interrupts occur much faster and the code is more complicated, so each stepper gets its own interrupt
void m3StepperInterrupt(void);
void m4StepperInterrupt(void);

/* Teensy setup */
void setup() {
  pinSetup(); // initializes all the appropriate pins to outputs or interrupt pins etc
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  UART_PORT.begin(BAUD_RATE);
  UART_PORT.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms
#elif defined(DEBUG_MODE) || defined(USER_MODE)
  nh.initNode();
  nh.subscribe(cmdSubscriber);
  nh.advertise(pub_m1);
  nh.advertise(pub_m2);
  nh.advertise(pub_m3);
  nh.advertise(pub_m4);
  nh.advertise(pub_m5);
  nh.advertise(pub_m6);

  m1_angle_msg.header.frame_id = m1FrameId;
  m2_angle_msg.header.frame_id = m2FrameId;
  m3_angle_msg.header.frame_id = m3FrameId;
  m4_angle_msg.header.frame_id = m4FrameId;
  m5_angle_msg.header.frame_id = m5FrameId;
  m6_angle_msg.header.frame_id = m6FrameId;
#endif

  // each motor with an encoder needs to attach the encoder and 2 interrupts
#ifdef M1_ENCODER_PORT
  motor1.attachEncoder(M1_ENCODER_A, M1_ENCODER_B, M1_ENCODER_PORT, M1_ENCODER_SHIFT, M1_ENCODER_RESOLUTION);
  // comment out the encoder pin which doesn't work, this is for testing purposes tho as the final one should have both channels working
  attachInterrupt(motor1.encoderPinA, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor1.encoderPinB, m1_encoder_interrupt, CHANGE);
  // motor1.pidController.setGainConstants(0.35,0.000001,15.0);
  motor1.pidController.setGainConstants(1.0, 0.0, 0.0);
#endif
#ifdef M2_ENCODER_PORT
  motor2.attachEncoder(M2_ENCODER_A, M2_ENCODER_B, M2_ENCODER_PORT, M2_ENCODER_SHIFT, M2_ENCODER_RESOLUTION);
  attachInterrupt(motor2.encoderPinA, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinB, m2_encoder_interrupt, CHANGE);
  motor2.pidController.setGainConstants(1.0, 0.0, 0.0);
#endif
#ifdef M3_ENCODER_PORT
  motor3.attachEncoder(M3_ENCODER_A, M3_ENCODER_B, M3_ENCODER_PORT, M3_ENCODER_SHIFT, M3_ENCODER_RESOLUTION);
  attachInterrupt(motor3.encoderPinA, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinB, m3_encoder_interrupt, CHANGE);
  motor3.pidController.setGainConstants(1.0, 0.0, 0.0);
#endif
#ifdef M4_ENCODER_PORT
  motor4.attachEncoder(M4_ENCODER_A, M4_ENCODER_B, M4_ENCODER_PORT, M4_ENCODER_SHIFT, M4_ENCODER_RESOLUTION);
  attachInterrupt(motor4.encoderPinA, m4_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinB, m4_encoder_interrupt, CHANGE);
  motor4.pidController.setGainConstants(1.0, 0.0, 0.0);
#endif
#ifdef M5_ENCODER_PORT
  motor5.attachEncoder(M5_ENCODER_A, M5_ENCODER_B, M5_ENCODER_PORT, M5_ENCODER_SHIFT, M5_ENCODER_RESOLUTION);
  attachInterrupt(motor5.encoderPinA, m5_encoder_interrupt, CHANGE);
  attachInterrupt(motor5.encoderPinB, m5_encoder_interrupt, CHANGE);
  motor5.pidController.setGainConstants(1.0, 0.0, 0.0);
#endif
#ifdef M6_ENCODER_PORT
  motor6.attachEncoder(M6_ENCODER_A, M6_ENCODER_B, M6_ENCODER_PORT, M6_ENCODER_SHIFT, M6_ENCODER_RESOLUTION);
  attachInterrupt(motor6.encoderPinA, m6_encoder_interrupt, CHANGE);
  attachInterrupt(motor6.encoderPinB, m6_encoder_interrupt, CHANGE);
  motor6.pidController.setGainConstants(1.0, 0.0, 0.0);
#endif

  // prepare and attach limit switch ISRs
  /*
    #if defined(LIM_SWITCH_FALL)
    #define LIM_SWITCH_DIR FALLING
    #elif defined(LIM_SWITCH_RISE)
    #define LIM_SWITCH_DIR RISING
    #endif
  */
#define LIM_SWITCH_DIR CHANGE

  motor1.attachLimitSwitches('c', M1_LIMIT_SW_CW, M1_LIMIT_SW_CCW);
  motor2.attachLimitSwitches('f', M2_LIMIT_SW_FLEX, M2_LIMIT_SW_EXTEND);
  motor3.attachLimitSwitches('f', M3_LIMIT_SW_FLEX, M3_LIMIT_SW_EXTEND);
  motor4.attachLimitSwitches('f', M4_LIMIT_SW_FLEX, M4_LIMIT_SW_EXTEND);
  //motor5.attachLimitSwitches('c', M5_LIMIT_SW_CW, M5_LIMIT_SW_CCW);
  //motor6.attachLimitSwitches('g', 0, M6_LIMIT_SW_EXTEND);
  attachInterrupt(motor1.limSwitchCw, m1CwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor1.limSwitchCcw, m1CcwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchFlex, m2FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchExtend, m2ExtendISR, LIM_SWITCH_DIR);
  attachInterrupt(motor3.limSwitchFlex, m3FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor3.limSwitchExtend, m3ExtendISR, LIM_SWITCH_DIR);
  attachInterrupt(motor4.limSwitchFlex, m4FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor4.limSwitchExtend, m4ExtendISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor5.limSwitchCw, m5CwISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor5.limSwitchCcw, m5CcwISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor6.limSwitchExtend, m6ExtendISR, LIM_SWITCH_DIR);
  // set motor shaft angle tolerances
  // motor1.pidController.setJointAngleTolerance(1.8 * 3*motor1.gearRatioReciprocal); // if it was a stepper
  motor1.pidController.setJointAngleTolerance(2.0 * motor1.gearRatioReciprocal); // randomly chosen for dc
  motor2.pidController.setJointAngleTolerance(2.0 * motor2.gearRatioReciprocal);
  motor3.pidController.setJointAngleTolerance(1.8 * 2 * motor3.gearRatioReciprocal); // 1.8 is the min stepper resolution so I gave it +/- tolerance
  motor4.pidController.setJointAngleTolerance(1.8 * 2 * motor4.gearRatioReciprocal);
  motor5.pidController.setJointAngleTolerance(2.0 * motor5.gearRatioReciprocal); // randomly chosen for servo
  motor6.pidController.setJointAngleTolerance(2.0 * motor6.gearRatioReciprocal);
  // set motor joint angle limits
  motor1.setAngleLimits(M1_MIN_HARD_ANGLE, M1_MAX_HARD_ANGLE, M1_MIN_SOFT_ANGLE, M1_MAX_SOFT_ANGLE);
  motor2.setAngleLimits(M2_MIN_HARD_ANGLE, M2_MAX_HARD_ANGLE, M2_MIN_SOFT_ANGLE, M2_MAX_SOFT_ANGLE);
  motor3.setAngleLimits(M3_MIN_HARD_ANGLE, M3_MAX_HARD_ANGLE, M3_MIN_SOFT_ANGLE, M3_MAX_SOFT_ANGLE);
  motor4.setAngleLimits(M4_MIN_HARD_ANGLE, M4_MAX_HARD_ANGLE, M4_MIN_SOFT_ANGLE, M4_MAX_SOFT_ANGLE);
  // motor5.setAngleLimits(M5_MIN_HARD_ANGLE, M5_MAX_HARD_ANGLE, M5_MIN_SOFT_ANGLE, M5_MAX_SOFT_ANGLE); // this joint should be able to spin freely
  motor6.setAngleLimits(M6_MIN_HARD_ANGLE, M6_MAX_HARD_ANGLE, M6_MIN_SOFT_ANGLE, M6_MAX_SOFT_ANGLE);
  // set max and min closed loop speeds (in percentage), I limit it to 50% for safety
  // Abtin thinks 50% should be a hard limit that can't be modified this easily
  motor1.pidController.setOutputLimits(-50, 50, 5.0);
  // motor2.pidController.setOutputLimits(-100, 100, 5.0);
  motor2.pidController.setOutputLimits(-50, 50, 5.0);
  motor3.pidController.setOutputLimits(-50, 50, 5.0);
  motor4.pidController.setOutputLimits(-50, 50, 5.0);
  motor5.pidController.setOutputLimits(-50, 50, 5.0);
  motor6.pidController.setOutputLimits(-50, 50, 5.0);
  // set open loop parameters. By default the motors are open loop, have constant velocity profiles (no ramping),
  // operate at 50% max speed, and the gains should vary based on which motor it is
  motor1.isOpenLoop = true;
  motor1.hasRamping = false;
  motor1.openLoopSpeed = 50; // 50% speed
  motor1.openLoopGain = 1.0; // totally random guess, needs to be tested
  motor2.isOpenLoop = true;
  motor2.hasRamping = false;
  motor2.openLoopSpeed = 25; // 50% speed
  motor2.openLoopGain = 0.01; // totally random guess, needs to be tested
  // open loop gain is only for time-based open loop control
  motor3.isOpenLoop = true;
  motor3.hasRamping = false;
  motor3.openLoopSpeed = 50; // 50% speed
  motor3.switchDirectionLogic(); // motor is wired backwards
  motor4.isOpenLoop = true;
  motor4.hasRamping = false;
  motor4.openLoopSpeed = 50; // 50% speed
  motor5.isOpenLoop = true;
  motor5.hasRamping = false;
  motor5.openLoopSpeed = 50; // 50% speed
  motor5.openLoopGain = 0.32; // for 5V
  motor5.switchDirectionLogic();
  motor6.isOpenLoop = true;
  motor6.hasRamping = false;
  motor6.openLoopSpeed = 50; // 50% speed
  motor6.openLoopGain = 0.35; // for 5V
  motor6.switchDirectionLogic(); // positive angles now mean opening
  // activate the timer interrupts
  m3StepperTimer.begin(m3StepperInterrupt, STEPPER_PID_PERIOD); // 1000ms
  m3StepperTimer.priority(MOTOR_NVIC_PRIORITY);
  m4StepperTimer.begin(m4StepperInterrupt, STEPPER_PID_PERIOD); // 1000ms
  m4StepperTimer.priority(MOTOR_NVIC_PRIORITY);
  dcTimer.begin(dcInterrupt, DC_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  dcTimer.priority(MOTOR_NVIC_PRIORITY);
  servoTimer.begin(servoInterrupt, SERVO_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  servoTimer.priority(MOTOR_NVIC_PRIORITY);
  // reset the elapsedMillis variables so that they're fresh upon entering the loop()
  sinceAnglePrint = 0;
  sinceStepperCheck = 0;
}

/* main code loop */
void loop() {

  // first check to see if limit switch interrupts have occurred, only then can i check if a message came in
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorArray[i]->triggered) {
      if (motorArray[i]->sinceTrigger >= TRIGGER_DELAY) {
        // if the last interrupt was a press (meaning it's stabilized and in contact)
        // then there's a real press
        if (motorArray[i]->triggerState != 0) {
          motorArray[i]->actualPress = true;
          motorArray[i]->limitSwitchState = motorArray[i]->triggerState;
        }
        // otherwise it's not a real press
        // so the limit switch state should stay whatever it used to be
        // and so should actualPress
        else {
          ;
        }
        // either way, we should reset the triggered bool in wait for the next trigger
        motorArray[i]->triggered = false;
      }
    }
    if (motorArray[i]->actualPress) {
#ifdef DEBUG_SWITCHES
      UART_PORT.print("motor ");
      UART_PORT.print(i+1);
      UART_PORT.print(" limit switch state ");
      UART_PORT.println(motorArray[i]->limitSwitchState);
#endif
      motorArray[i]->stopRotation();
      if (motorArray[i]->limitSwitchState == COUNTER_CLOCKWISE) {
#ifdef DEBUG_SWITCHES
        UART_PORT.print("motor ");
        UART_PORT.print(i+1);
        UART_PORT.println(" ccw");
#endif
        motorArray[i]->setSoftwareAngle(motorArray[i]->maxHardAngle);
        //motorArray[i]->goToAngle(motorArray[i]->maxSoftAngle);
      }
      if (motorArray[i]->limitSwitchState == CLOCKWISE) {
#ifdef DEBUG_SWITCHES
        UART_PORT.print("motor ");
        UART_PORT.print(i+1);
        UART_PORT.println(" cw");
#endif
        motorArray[i]->setSoftwareAngle(motorArray[i]->minHardAngle);
        //motorArray[i]->goToAngle(motorArray[i]->minSoftAngle);
      }
    }
    // now that the behaviour is complete we can reset these in wait for the next trigger to be confirmed
    motorArray[i]->actualPress = false;
    motorArray[i]->limitSwitchState = 0;
  }

  motorCommand = emptyMotorCommand; // reset motorCommand so the microcontroller doesn't try to move a motor next loop
  msgReceived = false;
  msgIsValid = false;
#if defined(DEBUG_MODE) || defined(USER_MODE)
  nh.spinOnce();
  if (msgReceived) {
    nh.logdebug(serialBuffer);
  }
#elif defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  if (UART_PORT.available()) {
    // if a message was sent to the Teensy
    msgReceived = true;
    UART_PORT.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
#ifdef DEBUG_MAIN
    UART_PORT.println(messageBar);
    UART_PORT.print("GOT: ");
    UART_PORT.println(serialBuffer); // send back what was received
#endif
    Parser.parseCommand(motorCommand, serialBuffer);
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    msgIsValid = Parser.verifCommand(motorCommand);
  }
#endif
  if (msgReceived) {
    if (msgIsValid) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#ifdef DEBUG_MAIN
      UART_PORT.println(messageBar);
#endif
#endif
      if (motorCommand.pingCommand) {
        // respond to ping
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("pong");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("pong");
#endif
      }
      else if (motorCommand.stopAllMotors) { // emergency stop takes precedence
        for (int i = 0; i < NUM_MOTORS; i++) {
          motorArray[i] -> stopRotation();
        }
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("all motors stopped because of emergency stop");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("all motors stopped because of emergency stop");
#endif
      }
      else if (motorCommand.resetAllMotors) { // emergency stop takes precedence
        for (int i = 0; i < NUM_MOTORS; i++) {
          motorArray[i] -> setImaginedAngle(0.0);
        }
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("all motor angle values reset");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("all motor angle values reset");
#endif
      }
      else { // following cases are for commands to specific motors
        if (motorCommand.stopSingleMotor) { // stopping a single motor takes precedence
          motorArray[motorCommand.whichMotor - 1] -> stopRotation();
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.print("stopped motor ");
          UART_PORT.println(motorCommand.whichMotor);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
          // this is SUPER DUPER GROSS
          String infoMessage = "stopped motor " + motorCommand.whichMotor;
          char actualMessage[50];
          for (unsigned int i = 0; i < infoMessage.length(); i++) {
            actualMessage[i] = infoMessage[i];
          }
          nh.loginfo(actualMessage);
#endif
        }
        else if (motorCommand.loopCommand) { // set loop states for appropriate motor
          if (motorCommand.loopState == OPEN_LOOP) {
            motorArray[motorCommand.whichMotor - 1] -> isOpenLoop = true;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("motor ");
            UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.println(" is open loop");
#endif
          }
          else if (motorCommand.loopState == CLOSED_LOOP) {
            if (motorArray[motorCommand.whichMotor - 1] -> hasEncoder) {
              motorArray[motorCommand.whichMotor - 1] -> isOpenLoop = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("motor ");
              UART_PORT.print(motorCommand.whichMotor);
              UART_PORT.println(" is closed loop");
#endif
            }
            else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
#endif
            }
          }
        }
        else if (motorCommand.resetCommand) { // reset the motor angle's variable or actually control the motor to reset it to neutral position
          if (motorCommand.resetAngleValue) {
            motorArray[motorCommand.whichMotor - 1] -> setImaginedAngle(0.0);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("reset angle value of motor ");
            UART_PORT.println(motorCommand.whichMotor);
#endif
          }
          else if (motorCommand.resetJointPosition) {
            ; // for later
          }
        }
        else if (motorCommand.switchDir) { // change the direction modifier to swap rotation direction in the case of backwards wiring
          motorArray[motorCommand.whichMotor - 1] -> switchDirectionLogic();
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.print("direction modifier is now ");
          UART_PORT.println(motorArray[motorCommand.whichMotor - 1] -> getDirectionLogic());
#endif
        }
        else if (motorCommand.budgeCommand) { // make motors move until the command isn't sent anymore
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (motorCommand.motorsToMove[i]) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("motor ");
              UART_PORT.print(i + 1);
              UART_PORT.print(" desired direction is: ");
              UART_PORT.println(motorCommand.directionsToMove[i]);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
              // this is SUPER DUPER GROSS
              int tempVal = i + 1;
              String infoMessage = "motor " + tempVal;
              infoMessage += " desired direction is: ";
              infoMessage += motorCommand.directionsToMove[i];
              char actualMessage[60];
              for (unsigned int i = 0; i < infoMessage.length(); i++) {
                actualMessage[i] = infoMessage[i];
              }
              nh.loginfo(actualMessage);
#endif
            }
          }
        }
        else if (motorCommand.multiMove) { // make motors move simultaneously
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (motorCommand.motorsToMove[i]) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("motor ");
              UART_PORT.print(i + 1);
              UART_PORT.print(" desired angle (degrees) is: ");
              UART_PORT.println(motorCommand.anglesToReach[i]);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
              // this is SUPER DUPER GROSS
              int tempVal = i + 1;
              String infoMessage = "motor " + tempVal;
              infoMessage += " desired angle (degrees) is: ";
              infoMessage += motorCommand.anglesToReach[i];
              char actualMessage[50];
              for (unsigned int i = 0; i < infoMessage.length(); i++) {
                actualMessage[i] = infoMessage[i];
              }
              nh.loginfo(actualMessage);
#endif
            }
            else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("motor ");
              UART_PORT.print(i + 1);
              UART_PORT.println(" will not change course");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
              // this is SUPER DUPER GROSS
              int tempVal = i + 1;
              String infoMessage = "motor " + tempVal;
              infoMessage += " will not change course";
              char actualMessage[50];
              for (unsigned int i = 0; i < infoMessage.length(); i++) {
                actualMessage[i] = infoMessage[i];
              }
              nh.loginfo(actualMessage);
#endif
            }
          }
        }
        bool motorsCanMove = true;
        for (int i = 0; i < NUM_MOTORS; i++) {
          if (motorCommand.budgeCommand) {
            motorArray[i] -> calcCurrentAngle();
            int dir = motorCommand.directionsToMove[i];
            float ang;
            if ( motorArray[i] -> isOpenLoop ) {
              ang = motorArray[i] -> getImaginedAngle();
            }
            else if ( !(motorArray[i] -> isOpenLoop) ) {
              ang = motorArray[i] -> getCurrentAngle();
            }
            if (motorArray[i] -> hasAngleLimits) {
              if ( ( (dir > 0) && (ang > motorArray[i] -> maxJointAngle) ) || ( (dir < 0) && (ang < motorArray[i] -> minJointAngle) ) ) {
                motorsCanMove = false;
              }
            }
          }
          else if (!(motorArray[i] -> withinJointAngleLimits(motorCommand.anglesToReach[i]))) {
            motorsCanMove = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("$E,Error: requested motor ");
            UART_PORT.print(i + 1);
            UART_PORT.println(" angle is not within angle limits.");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
            // this is SUPER DUPER GROSS
            int tempVal = i + 1;
            String infoMessage = "motor " + tempVal;
            infoMessage += " angle is not within angle limits";
            char actualMessage[50];
            for (unsigned int i = 0; i < infoMessage.length(); i++) {
              actualMessage[i] = infoMessage[i];
            }
            nh.logerror(actualMessage);
#endif
          }
        }
        if (!motorsCanMove) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.println("$E,Error: one or many angles are invalid, arm will not move");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
          nh.logerror("one or many angles are invalid, arm will not move");
#endif
        }
        else { // start moving things
#ifdef DEBUG_MAIN
          UART_PORT.println(messageBar);
          UART_PORT.println("$S,Success: all angles are valid, arm to move");
#endif
          if (motorCommand.motorsToMove[0]) { // this motor can swivel but has limits so it doesn't hit anything
            if (motorCommand.budgeCommand) {
              motor1.calcDirection(motorCommand.directionsToMove[0]); // yes, this works
              motor1.isBudging = true;
              motor1.movementDone = false;
              motor1.sinceBudgeCommand = 0;
              if (motor1.isOpenLoop) {
                motor1.startAngle = motor1.getImaginedAngle();
              }
              else if (!motor1.isOpenLoop) {
                motor1.startAngle = motor1.getCurrentAngle();
              }
            }
            else if (motor1.setDesiredAngle(motorCommand.anglesToReach[0])) { // this method returns true if the command is within joint angle limits
              if (motor1.isOpenLoop) {
                motor1.calcCurrentAngle();
                motor1.startAngle = motor1.getImaginedAngle();
                motor1.openLoopError = motor1.getDesiredAngle() - motor1.getImaginedAngle(); // find the angle difference
                motor1.calcDirection(motor1.openLoopError); // determine rotation direction and save the value
                // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
                if (motor1.calcTurningDuration(motor1.openLoopError)) { // returns false if the open loop error is too small
                  motor1.timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
                  motor1.movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.print("$S,Success: motor ");
                  UART_PORT.print(1);
                  UART_PORT.print(" to turn for ");
                  UART_PORT.print(motor1.numMillis);
                  UART_PORT.println(" milliseconds");
#endif
                }
                else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
                }
              }
              else if (!motor1.isOpenLoop) {
                // all the heavy lifting for closed loop control is done in the timer interrupt
                motor1.movementDone = false;
              }
            }
          }
          if (motorCommand.motorsToMove[1]) {
            if (motorCommand.budgeCommand) {
              motor2.calcDirection(motorCommand.directionsToMove[1]); // yes, this works
              motor2.isBudging = true;
              motor2.movementDone = false;
              motor2.sinceBudgeCommand = 0;
              if (motor2.isOpenLoop) {
                motor2.startAngle = motor2.getImaginedAngle();
              }
              else if (!motor2.isOpenLoop) {
                motor2.startAngle = motor2.getCurrentAngle();
              }
            }
            else if (motor2.setDesiredAngle(motorCommand.anglesToReach[1])) {
              if (motor2.isOpenLoop) {
                motor2.calcCurrentAngle();
                motor2.startAngle = motor2.getImaginedAngle();
                motor2.openLoopError = motor2.getDesiredAngle() - motor2.getImaginedAngle();
                motor2.calcDirection(motor2.openLoopError);
                if (motor2.calcTurningDuration(motor2.openLoopError)) {
                  motor2.timeCount = 0;
                  motor2.movementDone = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.print("$S,Success: motor ");
                  UART_PORT.print(2);
                  UART_PORT.print(" to turn for ");
                  UART_PORT.print(motor2.numMillis);
                  UART_PORT.println(" milliseconds");
#endif
                }
                else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
                }
              }
              else if (!motor2.isOpenLoop) {
                motor2.movementDone = false;
              }
            }
            else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.println("$E,Error: requested angle is not within angle limits.");
#endif
            }
          }
          if (motorCommand.motorsToMove[2]) {
            if (motorCommand.budgeCommand) {
              motor3.calcDirection(motorCommand.directionsToMove[2]); // yes, this works
              motor3.isBudging = true;
              motor3.movementDone = false;
              motor3.stepCount = 0;
              motor3.sinceBudgeCommand = 0;
              motor3.enablePower(); // give power to the stepper finally
              if (motor3.isOpenLoop) {
                motor3.startAngle = motor3.getImaginedAngle();
              }
              else if (!motor3.isOpenLoop) {
                motor3.startAngle = motor3.getCurrentAngle();
              }
            }
            else if (motor3.setDesiredAngle(motorCommand.anglesToReach[2])) {
              if (motor3.isOpenLoop) {
                motor3.calcCurrentAngle();
                motor3.startAngle = motor3.getImaginedAngle();
                motor3.openLoopError = motor3.getDesiredAngle() - motor3.getImaginedAngle(); // find the angle difference
                motor3.calcDirection(motor3.openLoopError);
                // calculates how many steps to take to get to the desired position, assuming no slipping
                if (motor3.calcNumSteps(motor3.openLoopError)) { // returns false if the open loop error is too small
                  motor3.stepCount = 0;
                  motor3.enablePower(); // give power to the stepper finally
                  motor3.movementDone = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.print("$S,Success: motor ");
                  UART_PORT.print(3);
                  UART_PORT.print(" to turn ");
                  UART_PORT.print(motor3.numSteps);
                  UART_PORT.println(" steps");
#endif
                }
                else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
                }
              }
              else if (!motor3.isOpenLoop) {
                motor3.enablePower();
                motor3.movementDone = false;
              }
            }
          }
          if (motorCommand.motorsToMove[3]) {
            if (motorCommand.budgeCommand) {
              motor4.calcDirection(motorCommand.directionsToMove[3]); // yes, this works
              motor4.isBudging = true;
              motor4.movementDone = false;
              motor4.stepCount = 0;
              motor4.sinceBudgeCommand = 0;
              motor4.enablePower();
              if (motor4.isOpenLoop) {
                motor4.startAngle = motor4.getImaginedAngle();
              }
              else if (!motor4.isOpenLoop) {
                motor4.startAngle = motor4.getCurrentAngle();
              }
            }
            else if (motor4.setDesiredAngle(motorCommand.anglesToReach[3])) {
              if (motor4.isOpenLoop) {
                motor4.calcCurrentAngle();
                motor4.startAngle = motor4.getImaginedAngle();
                motor4.openLoopError = motor4.getDesiredAngle() - motor4.getImaginedAngle(); // find the angle difference
                motor4.calcDirection(motor4.openLoopError);
                if (motor4.calcNumSteps(motor4.openLoopError)) {
                  // I don't set stepCount to 0?
                  motor4.enablePower();
                  motor4.movementDone = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.print("$S,Success: motor ");
                  UART_PORT.print(4);
                  UART_PORT.print(" to turn ");
                  UART_PORT.print(motor4.numSteps);
                  UART_PORT.println(" steps");
#endif
                }
                else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
                }
              }
              else if (!motor4.isOpenLoop) {
                motor4.enablePower();
                motor4.movementDone = false;
              }
            }
          }
          if (motorCommand.motorsToMove[4]) {
            // this motor has no max or min angle because it must be able to spin like a screwdriver
            if (motorCommand.budgeCommand) {
              motor5.calcDirection(motorCommand.directionsToMove[4]); // yes, this works
              motor5.isBudging = true;
              motor5.movementDone = false;
              motor5.sinceBudgeCommand = 0;
              if (motor5.isOpenLoop) {
                motor5.startAngle = motor5.getImaginedAngle();
              }
              else if (!motor5.isOpenLoop) {
                motor5.startAngle = motor5.getCurrentAngle();
              }
            }
            else if (motor5.setDesiredAngle(motorCommand.anglesToReach[4])) {
              if (motor5.isOpenLoop) {
                motor5.calcCurrentAngle();
                motor5.startAngle = motor5.getImaginedAngle();
                motor5.openLoopError = motor5.getDesiredAngle() - motor5.getImaginedAngle(); // find the angle difference
                motor5.calcDirection(motor5.openLoopError);
                if (motor5.calcTurningDuration(motor5.openLoopError)) {
                  motor5.timeCount = 0;
                  motor5.movementDone = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.print("$S,Success: motor ");
                  UART_PORT.print(5);
                  UART_PORT.print(" to turn for ");
                  UART_PORT.print(motor5.numMillis);
                  UART_PORT.println(" milliseconds");
#endif
                }
                else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
                }
              }
              else {
                if (!motor5.isOpenLoop) {
                  motor5.movementDone = false;
                }
              }
            }
          }
          if (motorCommand.motorsToMove[5]) {
            if (motorCommand.budgeCommand) {
              motor6.calcDirection(motorCommand.directionsToMove[5]); // yes, this works
              motor6.isBudging = true;
              motor6.movementDone = false;
              motor6.sinceBudgeCommand = 0;
              if (motor6.isOpenLoop) {
                motor6.startAngle = motor6.getImaginedAngle();
              }
              else if (!motor6.isOpenLoop) {
                motor6.startAngle = motor6.getCurrentAngle();
              }
            }
            else if (motor6.setDesiredAngle(motorCommand.anglesToReach[5])) {
              if (motor6.isOpenLoop) {
                motor6.calcCurrentAngle();
                motor6.startAngle = motor6.getCurrentAngle();
                motor6.openLoopError = motor6.getDesiredAngle() - motor6.getImaginedAngle(); // find the angle difference
                motor6.calcDirection(motor6.openLoopError);
                if (motor6.calcTurningDuration(motor6.openLoopError)) {
                  motor6.timeCount = 0;
                  motor6.movementDone = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.print("$S,Success: motor ");
                  UART_PORT.print(6);
                  UART_PORT.print(" to turn for ");
                  UART_PORT.print(motor6.numMillis);
                  UART_PORT.println(" milliseconds");
#endif
                }
                else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                  UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
                }
              }
              else if (!motor6.isOpenLoop) {
                motor6.movementDone = false;
              }
            }
          }
        }
      }
    }
    else { // bad command
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.println("$E,Error: bad motor command");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
      nh.logerror("error: bad motor command");
#endif
    }
  }
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) { // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }
}

void printMotorAngles(void) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  UART_PORT.print("Motor Angles: ");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorArray[i] -> isOpenLoop) {
      UART_PORT.print(motorArray[i] -> getImaginedAngle());
    }
    else {
      UART_PORT.print(motorArray[i] -> getCurrentAngle());
    }
    UART_PORT.print(", ");
  }
  UART_PORT.println("");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
  float angles[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorArray[i] -> isOpenLoop) {
      angles[i] = motorArray[i] -> getImaginedAngle();
      angleMessages[i].position = &(angles[i]);
      //*(angleMessages[i].position) = motorArray[i] -> getImaginedAngle();
    }
    else {
      angles[i] = motorArray[i] -> getCurrentAngle();
      angleMessages[i].position = &(angles[i]);
      //*(angleMessages[i].position) = motorArray[i] -> getCurrentAngle();
    }
  }
  pub_m1.publish(&m1_angle_msg);
  pub_m2.publish(&m2_angle_msg);
  pub_m3.publish(&m3_angle_msg);
  pub_m4.publish(&m4_angle_msg);
  pub_m5.publish(&m5_angle_msg);
  pub_m6.publish(&m6_angle_msg);
  nh.spinOnce(); // does it cause problems if i spin twice in loop()
#endif
}

void m3StepperInterrupt(void) {
  motor3.nextInterval = STEPPER_PID_PERIOD; // how long until the next step is taken? indirectly controls speed
  if (motor3.isBudging) {
    if (motor3.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor3.calcCurrentAngle();
      motor3.setVelocity(motor3.rotationDirection, motor3.openLoopSpeed);
      motor3.stepCount++;
      m3StepperTimer.update(motor3.nextInterval);
    }
    else {
      motor3.stepCount = 0; // reset the counter
      motor3.isBudging = false;
      motor3.movementDone = true;
      motor3.stopRotation();
      motor3.nextInterval = STEPPER_PID_PERIOD; // set it back to default
    }
  }
  else if (motor3.isOpenLoop) { // open loop control
    // movementDone can be set elsewhere... so can numSteps
    if (!motor3.movementDone && motor3.stepCount < motor3.numSteps) {
      motor3.calcCurrentAngle();
      motor3.setVelocity(motor3.rotationDirection, motor3.openLoopSpeed); // direction was set beforehand
      motor3.stepCount++;
      if (motor3.hasRamping) {
        // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        // nextInterval = STEPPER_PID_PERIOD; // replace with accessing a motor-specific array
      }
      m3StepperTimer.update(motor3.nextInterval); // sets the new period for the timer interrupt
#ifdef DEBUG_STEPPER_3_TIMER
      UART_PORT.println("motor 3");
      UART_PORT.print(motor3.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor3.stepCount); UART_PORT.print("\t / ");
      UART_PORT.print(motor3.numSteps); UART_PORT.println(" steps");
      UART_PORT.print(motor3.getImaginedAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor3.getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor3.startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor3.stepCount = 0; // reset the counter
      motor3.movementDone = true;
      motor3.stopRotation();
      motor3.nextInterval = STEPPER_PID_PERIOD; // set it back to default
    }
  }
  else if (!motor3.isOpenLoop) { // closed loop control
    if (!motor3.movementDone) {
      motor3.calcCurrentAngle();
      // determine the speed of the motor for next motor step
      float output = motor3.pidController.updatePID(motor3.getCurrentAngle(), motor3.getDesiredAngle());
      if (output == 0) {
        motor3.movementDone = true;
        motor3.stopRotation();
        motor3.nextInterval = STEPPER_PID_PERIOD; // set it back to default
      }
      else {
        int dir = motor3.calcDirection(output);
        // makes the motor step and calculates how long to wait until the next motor step
        motor3.setVelocity(dir, output);
        m3StepperTimer.update(motor3.nextInterval); // sets the new period for the timer interrupt
      }
#ifdef DEBUG_STEPPER_3_TIMER
      UART_PORT.println("motor 3");
      UART_PORT.print(motor3.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor3.nextInterval);
      UART_PORT.println(" next interval");
#endif
    }
    else {
      motor3.stopRotation();
    }
  }
}

void m4StepperInterrupt(void) {
  motor4.nextInterval = STEPPER_PID_PERIOD;
  if (motor4.isBudging) {
    if (motor4.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor4.calcCurrentAngle();
      motor4.setVelocity(motor4.rotationDirection, motor4.openLoopSpeed);
      motor4.stepCount++;
      m4StepperTimer.update(motor4.nextInterval);
    }
    else {
      motor4.isBudging = false;
      motor4.movementDone = true;
      motor4.stopRotation();
    }
  }
  else if (motor4.isOpenLoop) {
    if (!motor4.movementDone && motor4.stepCount < motor4.numSteps) {
      motor4.calcCurrentAngle();
      motor4.setVelocity(motor4.rotationDirection, motor4.openLoopSpeed); // direction was set beforehand
      motor4.stepCount++;
      if (motor4.hasRamping) {
        // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        // nextInterval = STEPPER_PID_PERIOD; // replace with accessing a motor-specific array
      }
      m4StepperTimer.update(motor4.nextInterval);
#ifdef DEBUG_STEPPER_4_TIMER
      UART_PORT.println("motor 4");
      UART_PORT.print(motor4.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor4.stepCount); UART_PORT.print("\t / ");
      UART_PORT.print(motor4.numSteps); UART_PORT.println(" steps");
      UART_PORT.print(motor4.getImaginedAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor4.getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor4.startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      motor4.stepCount = 0;
      motor4.movementDone = true;
      motor4.stopRotation();
      motor4.nextInterval = STEPPER_PID_PERIOD;
    }
  }
  else if (!motor4.isOpenLoop) {
    if (!motor4.movementDone) {
      motor4.calcCurrentAngle();
      float output = motor4.pidController.updatePID(motor4.getCurrentAngle(), motor4.getDesiredAngle());
      if (output == 0) {
        motor4.movementDone = true;
        motor4.stopRotation();
        motor4.nextInterval = STEPPER_PID_PERIOD;
      }
      else {
        int dir = motor4.calcDirection(output);
        motor4.setVelocity(dir, output);
        m4StepperTimer.update(motor4.nextInterval);
      }
#ifdef DEBUG_STEPPER_4_TIMER
      UART_PORT.println("motor 4");
      UART_PORT.print(motor4.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor4.nextInterval);
      UART_PORT.println(" next interval");
#endif
    }
    else {
      motor4.stopRotation();
    }
  }
}

void dcInterrupt(void) {
  if (motor1.isBudging) {
    if (motor1.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor1.calcCurrentAngle();
      motor1.setVelocity(motor1.rotationDirection, motor1.openLoopSpeed);
    }
    else {
      motor1.isBudging = false;
      motor1.movementDone = true;
      motor1.stopRotation();
    }
  }
  // movementDone can be set elsewhere... so can numMillis, openLoopSpeed and rotationDirection (in open loop control)
  else if (motor1.isOpenLoop) { // open loop control
    if (!motor1.movementDone && motor1.timeCount <= motor1.numMillis) {
      // calculates the pwm to send to the motor and makes it move
      motor1.calcCurrentAngle();
      motor1.setVelocity(motor1.rotationDirection, motor1.openLoopSpeed);
#ifdef DEBUG_DC_TIMER
      UART_PORT.println("motor 1");
      UART_PORT.print(motor1.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor1.timeCount); UART_PORT.print("\t / ");
      UART_PORT.print(motor1.numMillis); UART_PORT.println(" ms");
      UART_PORT.print(motor1.getImaginedAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor1.getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor1.startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      motor1.movementDone = true;
      motor1.stopRotation();
    }
  }
  // would be nice to have some kind of check for the above functions so the command only runs if there's been a change
  // e.g. movementDone changed or the speed or numMillis changed
  else if (!motor1.isOpenLoop) {
    if (!motor1.movementDone) {
      motor1.calcCurrentAngle();
      // determine the speed of the motor until the next interrupt
      float output = motor1.pidController.updatePID(motor1.getCurrentAngle(), motor1.getDesiredAngle());
      if (output == 0) {
        motor1.movementDone = true;
        motor1.stopRotation();
      }
      else {
        int dir = motor1.calcDirection(output);
        // calculates the pwm to send to the motor and makes it move
        motor1.setVelocity(dir, output);
#ifdef DEBUG_DC_TIMER
        UART_PORT.println("motor 1");
        UART_PORT.print(motor1.rotationDirection);
        UART_PORT.println(" direction");
        UART_PORT.print(output);
        UART_PORT.println(" next output");
#endif
      }
    }
    else {
      motor1.stopRotation();
    }
  }
  if (motor2.isBudging) {
    if (motor2.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor2.calcCurrentAngle();
      motor2.setVelocity(motor2.rotationDirection, motor2.openLoopSpeed);
    }
    else {
      motor2.isBudging = false;
      motor2.movementDone = true;
      motor2.stopRotation();
    }
  }
  else if (motor2.isOpenLoop) {
    if (!motor2.movementDone && motor2.timeCount <= motor2.numMillis) {
      motor2.calcCurrentAngle();
      motor2.setVelocity(motor2.rotationDirection, motor2.openLoopSpeed);
#ifdef DEBUG_DC_TIMER
      UART_PORT.println("motor 2");
      UART_PORT.print(motor2.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor2.timeCount); UART_PORT.print("\t / ");
      UART_PORT.print(motor2.numMillis); UART_PORT.println(" ms");
      UART_PORT.print(motor2.getImaginedAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor2.getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor2.startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      motor2.movementDone = true;
      motor2.stopRotation();
    }
  }
  else if (!motor2.isOpenLoop) {
    if (!motor2.movementDone) {
      motor2.calcCurrentAngle();
      float output = motor2.pidController.updatePID(motor2.getCurrentAngle(), motor2.getDesiredAngle());
      if (output == 0) {
        motor2.movementDone = true;
        motor2.stopRotation();
      }
      else {
        int dir = motor2.calcDirection(output);
        motor2.setVelocity(dir, output);
#ifdef DEBUG_DC_TIMER
        UART_PORT.println("motor 2");
        UART_PORT.print(motor2.rotationDirection);
        UART_PORT.println(" direction");
        UART_PORT.print(output);
        UART_PORT.println(" next output");
#endif
      }
    }
    else {
      motor2.stopRotation();
    }
  }
}

void servoInterrupt(void) {
  // movementDone can be set elsewhere... so are numMillis,
  // openLoopSpeed and rotationDirection (in open loop control)numMillis
  // motor 5
  if (motor5.isBudging) {
    if (motor5.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor5.calcCurrentAngle();
      motor5.setVelocity(motor5.rotationDirection, motor5.openLoopSpeed);
    }
    else {
      motor5.isBudging = false;
      motor5.movementDone = true;
      motor5.stopRotation();
    }
  }
  else if (motor5.isOpenLoop) {
    // open loop control
    if (!motor5.movementDone && motor5.timeCount < motor5.numMillis) {
      motor5.calcCurrentAngle();
      motor5.setVelocity(motor5.rotationDirection, motor5.openLoopSpeed);
#ifdef DEBUG_SERVO_TIMER
      UART_PORT.println("motor 5");
      UART_PORT.print(motor5.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor5.timeCount); UART_PORT.print("\t / ");
      UART_PORT.print(motor5.numMillis); UART_PORT.println(" ms");
      UART_PORT.print(motor5.getImaginedAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor5.getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor5.startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor5.movementDone = true;
      motor5.stopRotation();
    }
  }
  else if (!motor5.isOpenLoop) {
    if (!motor5.movementDone) {
      motor5.calcCurrentAngle();
      float output = motor5.pidController.updatePID(motor5.getCurrentAngle(), motor5.getDesiredAngle());
      if (output == 0) {
        motor5.movementDone = true;
        motor5.stopRotation();
      }
      else {
        int dir = motor5.calcDirection(output);
        motor5.setVelocity(dir, output);
#ifdef DEBUG_DC_TIMER
        UART_PORT.println("motor 5");
        UART_PORT.print(motor5.rotationDirection);
        UART_PORT.println(" direction");
        UART_PORT.print(output);
        UART_PORT.println(" next output");
#endif
      }
    }
    else {
      motor5.stopRotation();
    }
  }
  // motor 6
  if (motor6.isBudging) {
    if (motor6.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor6.calcCurrentAngle();
      motor6.setVelocity(motor6.rotationDirection, motor6.openLoopSpeed);
    }
    else {
      motor6.isBudging = false;
      motor6.movementDone = true;
      motor6.stopRotation();
    }
  }
  else if (motor6.isOpenLoop) { // open loop control
    if (!motor6.movementDone && motor6.timeCount < motor6.numMillis) {
      motor6.calcCurrentAngle();
      motor6.setVelocity(motor6.rotationDirection, motor6.openLoopSpeed);
#ifdef DEBUG_SERVO_TIMER
      UART_PORT.println("motor 6");
      UART_PORT.print(motor6.rotationDirection);
      UART_PORT.println(" direction");
      UART_PORT.print(motor6.timeCount); UART_PORT.print("\t / ");
      UART_PORT.print(motor6.numMillis); UART_PORT.println(" ms");
      UART_PORT.print(motor6.getImaginedAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor6.getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(motor6.startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor6.movementDone = true;
      motor6.stopRotation();
    }
  }
  else if (!motor6.isOpenLoop) {
    if (!motor6.movementDone) {
      motor6.calcCurrentAngle();
      float output = motor6.pidController.updatePID(motor6.getCurrentAngle(), motor6.getDesiredAngle());
      if (output == 0) {
        motor6.movementDone = true;
        motor6.stopRotation();
      }
      else {
        int dir = motor6.calcDirection(output);
        motor6.setVelocity(dir, output);
#ifdef DEBUG_DC_TIMER
        UART_PORT.println("motor 6");
        UART_PORT.print(motor6.rotationDirection);
        UART_PORT.println(" direction");
        UART_PORT.print(output);
        UART_PORT.println(" next output");
#endif
      }
    }
    else {
      motor6.stopRotation();
    }
  }
}

/* encoder ISRs */

#ifdef M1_ENCODER_PORT
/*
  void m1_encoder_interrupt(void) {
  // encoder states are 4 bit values
  // top 2 bits are the previous states of encoder channels A and B, bottom 2 are current states
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2; // shift current state into previous state
  // put the 2 relevant pin states from the relevant gpio port into memory, clearing the irrelevant bits
  // this is done by 1) grabbing the input register of the port,
  // 2) shifting it until the relevant bits are in the lowest state,
  // and 3) clearing all the bits higher than the lowest 2
  // next, place said (current) pin states into the bottom 2 bits of oldEncoderState
  oldEncoderState |= ((M1_ENCODER_PORT >> M1_ENCODER_SHIFT) & 0x03);
  // the dir[] array corresponds to the correct direction for a specific set of prev and current encoder states
  // the & operation ensures that anything above the lowest 4 bits is cleared before accessing the array
  motor1.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
  #ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 1 ");
  UART_PORT.println(motor1.encoderCount);
  #endif
  }
*/
void m1_encoder_interrupt(void) {
  // if only one channel is working, the same interrupt means more angle attained
  motor1.encoderCount += motor1.rotationDirection * 2;
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 1 ");
  UART_PORT.println(motor1.encoderCount);
#endif
}
#endif
#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 2 ");
  // UART_PORT.println(oldEncoderState,BIN);
  // UART_PORT.println(" ");
  UART_PORT.println(motor2.encoderCount);
#endif
}
#endif
#ifdef M3_ENCODER_PORT
void m3_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M3_ENCODER_PORT >> M3_ENCODER_SHIFT) & 0x03);
  motor3.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.println("motor 3 ");
  UART_PORT.println(motor3.encoderCount);
  // UART_PORT.println(M3_ENCODER_PORT,BIN);
#endif
}
#endif
#ifdef M4_ENCODER_PORT
void m4_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M4_ENCODER_PORT >> M4_ENCODER_SHIFT) & 0x03);
  motor4.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 4 ");
  UART_PORT.println(motor4.encoderCount);
#endif
}
#endif
#ifdef M5_ENCODER_PORT
void m5_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M5_ENCODER_PORT >> M5_ENCODER_SHIFT) & 0x03);
  motor5.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 5 ");
  UART_PORT.println(motor5.encoderCount);
#endif
}
#endif
#ifdef M6_ENCODER_PORT
void m6_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M6_ENCODER_PORT >> M6_ENCODER_SHIFT) & 0x03);
  motor6.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 6 ");
  UART_PORT.println(motor6.encoderCount);
#endif
}
#endif

/* limit switch ISRs */
void m1CwISR(void) {
  if (!(motor1.triggered) && motor1.triggerState == 0) {
    motor1.triggered = true;
    motor1.sinceTrigger = 0;
  }
  int pinState = (M1_LIMIT_SW_CW_PORT >> M1_LIMIT_SW_CW_SHIFT ) & 1;
  if (pinState == 0) {
    motor1.triggerState = CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("clockwise");
#endif
  }
  else {
    motor1.triggerState = 0;
  }
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
}
void m1CcwISR(void) {
  if (!(motor1.triggered) && motor1.triggerState == 0) {
    motor1.triggered = true;
    motor1.sinceTrigger = 0;
  }
  int pinState = (M1_LIMIT_SW_CCW_PORT >> M1_LIMIT_SW_CCW_SHIFT ) & 1;
  if (pinState == 0) {
    motor1.triggerState = COUNTER_CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("counter-clockwise");
#endif
  }
  else {
    motor1.triggerState = 0;
  }
}
void m2FlexISR(void) {
  if (!(motor2.triggered) && motor2.triggerState == 0) {
    motor2.triggered = true;
    motor2.sinceTrigger = 0;
  }
  int pinState = (M2_LIMIT_SW_FLEX_PORT >> M2_LIMIT_SW_FLEX_SHIFT ) & 1;
  if (pinState == 0) {
    motor2.triggerState = CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("clockwise");
#endif
  }
  else {
    motor2.triggerState = 0;
  }
}
void m2ExtendISR(void) {
  if (!(motor2.triggered) && motor2.triggerState == 0) {
    motor2.triggered = true;
    motor2.sinceTrigger = 0;
  }
  int pinState = (M2_LIMIT_SW_EXTEND_PORT >> M2_LIMIT_SW_EXTEND_SHIFT ) & 1;
  if (pinState == 0) {
    motor2.triggerState = COUNTER_CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("counter-clockwise");
#endif
  }
  else {
    motor2.triggerState = 0;
  }
}
void m3FlexISR(void) {
  if (!(motor3.triggered) && motor3.triggerState == 0) {
    motor3.triggered = true;
    motor3.sinceTrigger = 0;
  }
  int pinState = (M3_LIMIT_SW_FLEX_PORT >> M3_LIMIT_SW_FLEX_SHIFT ) & 1;
  if (pinState == 0) {
    motor3.triggerState = CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("clockwise");
#endif
  }
  else {
    motor3.triggerState = 0;
  }
}
void m3ExtendISR(void) {
  if (!(motor3.triggered) && motor3.triggerState == 0) {
    motor3.triggered = true;
    motor3.sinceTrigger = 0;
  }
  int pinState = (M3_LIMIT_SW_EXTEND_PORT >> M3_LIMIT_SW_EXTEND_SHIFT ) & 1;
  if (pinState == 0) {
    motor3.triggerState = COUNTER_CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("counter-clockwise");
#endif
  }
  else {
    motor3.triggerState = 0;
  }
}
void m4FlexISR(void) {
  if (!(motor4.triggered) && motor4.triggerState == 0) {
    motor4.triggered = true;
    motor4.sinceTrigger = 0;
  }
  int pinState = (M4_LIMIT_SW_FLEX_PORT >> M4_LIMIT_SW_FLEX_SHIFT ) & 1;
  if (pinState == 0) {
    motor4.triggerState = CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("clockwise");
#endif
  }
  else {
    motor4.triggerState = 0;
  }
}
void m4ExtendISR(void) {
  if (!(motor4.triggered) && motor4.triggerState == 0) {
    motor4.triggered = true;
    motor4.sinceTrigger = 0;
  }
  int pinState = (M4_LIMIT_SW_EXTEND_PORT >> M4_LIMIT_SW_EXTEND_SHIFT ) & 1;
  if (pinState == 0) {
    motor4.triggerState = COUNTER_CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("counter-clockwise");
#endif
  }
  else {
    motor4.triggerState = 0;
  }
}
/*
  void m5CcwISR(void) {
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
  }
  void m5FlexISR(void) {
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
  }
  void m6ExtendISR(void) {
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
  }
*/
