/*
  This is the main sketch which defines the control logic of the robotic arm of
  the Space Concordia Division, which runs on a Teensy 3.6 that will communicate
  with an Odroid XU4.
  The code can be compiled for serial communication over usb (if connected to a
  standard computer), or for serial communication over TX/RX pins (if connected
  to the Odroid). In the latter case, communication can be done either directly
  with Serial1, or with ROSserial, if integration in the ROS network is desired.
  Currently, this code is built for the control of six motors: two DC motors,
  two stepper motors, and two continuous rotation servos. Several helper classes
  were written to abstract away the complexities of controlling different types
  of motors and communicating with a master device.
  The code allows position control for all six of the arm's joints. It also
  will have a homing routine which depends on limit switches and sets the
  0 degrees position for each joint. The code also allows to stop motors at
  any point in time. Open loop control can be performed without the use of
  encoders but results in imprecise and jerky control. Use of ramping allows
  for less jerky control. The best control is closed loop control, which uses
  encoders for smooth speed profiles.
  The code starts by setting up a variety of events. It sets up the Teensy's
  GPIO pins, initializes the motor objects with the correct parameters (gear
  ratio, angle limits, etc), it starts up the timers and interrupt service
  routines, and it initializes communications with the master device.
  The main loop listens over the serial port for commands, parses them,
  then verifies the commands. Based on the type of command, the microcontroller
  will know how to control the motors.
  Variables changed in the main loop allow the timer interrupts to actually turn
  the motors, either in open loop or closed loop. The main loop can also perform
  periodic checks for motors in open loop, to effectuate small corrections to
  position during movements - if the appropriate motor has an encoder on it.
  This code began development sometime in July 2018 and is still being
  updated as of March 27 2019.
*/

/* still in idea phase */
#define DEVEL_MODE_1 1 //!< serial communication over USB, everything unlocked
//#define DEVEL_MODE_2 2 //!< serial communication over UART1, everything unlocked
//#define DEBUG_MODE 3 //!< ROSserial communication over UART1, everything unlocked
//#define USER_MODE 4 //!< ROSserial communication over UART1, functionality restricted
//#define ENABLE_ROS 5 //!< if testing on a computer without ROSserial, comment this to stop errors from rosserial not being installed. obviously you can't use ROSserial if that's the case

// debug statements shouldn't be sent if ros is working
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#define DEBUG_MAIN 10 //!< debug messages during main loop
//#define DEBUG_PARSING 11 //!< debug messages during parsing function
//#define DEBUG_VERIFYING 12 //!< debug messages during verification function
//#define DEBUG_ENCODERS 13 //!< debug messages during encoder interrupts
//#define DEBUG_PID 14 //!< debug messages during pid loop calculations
#define DEBUG_SWITCHES 15 //!< debug messages during limit switch interrupts
#define DEBUG_HOMING 16 //!< debug messages during homing sequence
//#define DEBUG_DC_TIMER 17 //!< debug messages during dc timer interrupts
//#define DEBUG_SERVO_TIMER 18 //!< debug messages during servo timer interrupts
//#define DEBUG_STEPPER_3_TIMER 19 //!< debug messages during stepper 3 timer interrupts
//#define DEBUG_STEPPER_4_TIMER 20 //!< debug messages during stepper 3 timer interrupts
#endif

/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
// serial communication over usb with pc, teensy not connected to odroid

#if defined(DEVEL_MODE_1)
#define UART_PORT Serial
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

#ifdef ENABLE_ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#endif

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
#define LIMIT_SWITCH_NVIC_PRIORITY 100 //!< limit switch interrupt priority is highest
#define ENCODER_NVIC_PRIORITY LIMIT_SWITCH_NVIC_PRIORITY + 4 //!< encoder interrupt priority is second highest
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4 //!< motor timer interrupt priority is third highest
/* serial */
#define BAUD_RATE 115200 //!< serial bit rate
#define SERIAL_PRINT_INTERVAL 1000 //!< how often should teensy send angle data
#define SERIAL_READ_TIMEOUT 50 //!< how often should the serial port be read
#define BUFFER_SIZE 100 //!< size of the buffer for the serial commands
/* comms */
char serialBuffer[BUFFER_SIZE]; //!< serial buffer used for early- and mid-stage testing without ROSserial

// kinda weird that this comes out of nowhere... maybe should be Parser.commandInfo or something. or define it here instead of parser
/*
  info from parsing functionality is packaged and given to motor control functionality.
  many of these are set to 0 so that the message can reset, thus making sure that
  the code later on doesn't inadvertently make a motor move when it wasn't supposed to
*/
commandInfo motorCommand; //!< struct which holds command data to be sent to main loop
commandInfo emptyMotorCommand; //!< used to reset the struct when the loop restarts
Parser Parser; //!< object which parses and verifies commands
bool msgReceived = false; //!< If true, the MCU will attempt to interpret a command.
bool msgIsValid = false; //!< If true, the MCU will execute a command. Otherwise, it will send an error message.

/*LED variables*/
/*
bool msgCheck = false; //!< If a message was received, while this remains true, the MCU will blink (but it's not a blocking while loop).
bool msgState = false; //!< If true, blinking will be in the success pattern. Otherwise it will be in the error pattern.

const int ledPin = 13; // note to nick: LED_BUILTIN is predefined in arduino C++ so just use that instead
unsigned long int previousMillis = 0; //stores previous time (in millis) LED was updated // note to nick: this variable name is too generic given how many other variables are used. Use more specific variable names, and also I recommend using elapsedMillis objects like I do to keep my code consistent
int ledState = LOW;

const int goodBlinkCounter = 4;
const int badBlinkCounter = 12;
const int goodBlinkInterval = 250;
const int badBlinkInterval = 100;
bool complete = false; // note to nick: this variable name is way too generic to just be lying around in the middle of all my code. please use something more specific
*/

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
#ifdef ENABLE_ROS
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

// these hold information that is sent from the teensy to ros
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
#endif

/* motors */
//! quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states
const int encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// instantiate motor objects here:
DcMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO);
DcMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO);
DcMotor motor3(M3_DIR_PIN, M3_PWM_PIN, M3_GEAR_RATIO);
StepperMotor motor4(M4_ENABLE_PIN, M4_DIR_PIN, M4_STEP_PIN, M4_STEP_RESOLUTION, FULL_STEP, M4_GEAR_RATIO);
ServoMotor motor5(M5_PWM_PIN, M5_GEAR_RATIO);
ServoMotor motor6(M6_PWM_PIN, M6_GEAR_RATIO);

// motor array prep work: making pointers to motor objects
DcMotor *m1 = &motor1; DcMotor *m2 = &motor2; DcMotor *m3 = &motor3;
StepperMotor *m4 = &motor4;
ServoMotor *m5 = &motor5; ServoMotor *m6 = &motor6;
RobotMotor *motorArray[] = {m1, m2, m3, m4, m5, m6}; //!< I can use this instead of switch/case statements by doing motorArray[motornumber]->attribute
// instantiate timers here:
IntervalTimer dcTimer; // motors 1,2&3
IntervalTimer m4StepperTimer; // motor 4
IntervalTimer servoTimer; // motors 5&6

// these are a nicer way of timing events than using millis()
elapsedMillis sinceAnglePrint; //!< how long since last time angle data was sent
elapsedMillis sinceStepperCheck; //!< how long since last time stepper angle was verified
// note to nick: i'd recommend putting your blinkled timer here

// homing variables
bool isHoming = false;
int homingMotor = NUM_MOTORS; //! initialize to a value that's invalid so it'll be ignored. Used in main loop to remember which motors need to be homed since commandInfo is reset each loop iteration
//! used in main loop to remember which motors need to be homed since commandInfo is reset each loop iteration
bool motorsToHome[] = {false, false, false, false, false, false}; 
/* function declarations */
void printMotorAngles(void); //!< sends all motor angles over serial
void initComms(void); //!< start up serial or usb communication
void initEncoders(void); //!< attach encoder interrupts and setup pid gains
void initLimitSwitches(void); //!< setup angle limits and attach limit switch interrupts
void initSpeedParams(void); //!< setup open and closed loop speed parameters
void initMotorTimers(void); //!< start the timers which control the motors

// all interrupt service routines (ISRs) must be global functions to work
// declare encoder interrupt service routines
void m1_encoder_interrupt(void);
void m2_encoder_interrupt(void);
void m3_encoder_interrupt(void);
void m4_encoder_interrupt(void);
void m5_encoder_interrupt(void);
void m6_encoder_interrupt(void);

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
//void m6OpenISR(void);

// declare timer interrupt service routines, where the motors actually get controlled
void dcInterrupt(void); //!< manages motors 1&2
/*! \brief manages motor 4
 * 
 * Stepper interrupts occur much faster and the code is more complicated, so each stepper needs its own interrupt.
*/
void m4StepperInterrupt(void);
void servoInterrupt(void); //!< manages motors 5&6

/*! \brief Teensy setup. Calls many init functions to prep comms and motors.
 * 
 * \todo Implement error checking, clean up code, comment code, make sure ISR variables are volatile
 * \todo Make sure the math calculations are written correctly and calculate as quickly as possible.
 * Floating point math doesn't seem bad, but at worst, convert float to int
 * before motor control and do int math inside interrupts.
 * \todo Fix up homing and limit switch functionality to deal with regions near angle limits
 * \todo (Nick) Finish implementing/integrating heartbeat and watchdog interrupt
 * \todo Figure out where to disable interrupts so that I don't read a value while it's being modified
 * \todo Determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
 * \todo Issue: interrupt functions must be defined outside of classes...
 * \todo Confirm all the pins will work with interrupts and not stepping on each other.
 * Do I send 3.3v to all interrupt pins simultaneously to test thing?
 * \todo Deal with overflow of encoderCount.. does it ever reach max value?
 * \todo What do I do for angles over 360? Do I keep counting up?
 * Do I keep count of how many rotations I've done?
 * \todo Different types of ramping profiles - trapezoid vs quintic polynomial?
 * Ramping of stepper should be linear, higher level ramping should occur in gui.
 * The base station/odroid should be in charge of ramping up angles and the teensy
 * should just go to them. Perhaps if a large angle is requested there should 
 * still be a way to stop it though.
 * \todo Determine whether it's worth it to use the built in quadrature decoders.
 * Quadrature on ftm1,2: pins 3/4,29/30: cant use for pwm anymore.
 * Quadrature on tpm1,2: pins 16/17, (tpm2 not implemented in teensy?).
 * \todo stepper motor angle checks for open loop control remains to be fixed, updated, implemented
 * \todo in stepper angle checks, also fix that issue with discrepancy
 */
void setup() {
  pinSetup();
  initComms();
  initEncoders();
  initLimitSwitches(); //!< \todo setJointAngleTolerance in here might need to be adjusted when gear ratio is adjusted!!! check other dependencies too!!!
  initSpeedParams();
  motor3.switchDirectionLogic(); // motor is wired backwards? replaced with dc, needs new test
  motor6.switchDirectionLogic(); // positive angles now mean opening
  initMotorTimers();

  // reset the elapsedMillis variables so that they're fresh upon entering the loop()
  sinceAnglePrint = 0; sinceStepperCheck = 0;
}

/*! \brief Main code which loops forever. Parses commands, prints motor angles and blinks the builtin LED.
 * 
 * \todo There should be a check so that if the motor is moving away from the goal position 
 * or has been moving for a while without reaching the expected angle, it stops...
 * like a timeout.
 * \todo What happens if a new command tells the motor to turn in opposite direction? Abrupt changes are bad.
 * \todo If the stepper is trying to turn but hasn't gotten anywhere, there should be
 * a check in the microcontroller that there's an issue (there can also be a check in the gui)
 * \todo Check to see if any global variables can be turned into static variables inside loop()
 * \todo I noticed that sending a new move command while motors are moving messes with open loop calculations?
 */
void loop() {
  /* limit switch checks occur before listening for commands */
  for (int i = 0; i < NUM_MOTORS; i++) { // I should maybe make a debouncer class?
    if (motorArray[i]->triggered) { // check if the switch was hit
      motorArray[i]->checkForActualPress();
    }
    if (motorArray[i]->actualPress) { // the switch was debounced and now we can react
#ifdef DEBUG_SWITCHES
      UART_PORT.print("motor "); UART_PORT.print(i + 1);
      UART_PORT.println(" hit limit switch");
#endif
      motorArray[i]->atSafeAngle = false;
      if (isHoming) {
        motorArray[i]->homingDone = true; // just means that the switch was hit so homing round 2 can start if desired
      }
      motorArray[i]->goToSafeAngle(); // internally stops movement and calls forceToAngle to overwrite previous command
    }
    /*! \todo put code here to check if the motor should be at the end of its path but isn't?
     * well how would it know if it isn't if it doesn't hit the limit switch because of software limits?
    */
  }

  /* Homing functionality ignores most message types */
  if (isHoming) { // not done homing the motors
    if (homingMotor < NUM_MOTORS) {
      if (motorsToHome[homingMotor]) { // is this motor supposed to home?
        // the homing direction should be set-able based on the homing command if single direction (or even both i guess)
#ifdef DEBUG_HOMING
        UART_PORT.print("homing motor "); UART_PORT.print(homingMotor + 1);
        UART_PORT.println(" inwards");
#endif
        motorArray[homingMotor]->homeMotor('i'); // start homing motor inwards
        motorsToHome[homingMotor] = false; // set this to false so it only happens once
      }
      if (motorArray[homingMotor]->homingDone) { // finished homing in a direction, set by motor timer interrupt
        if (motorArray[homingMotor]->atSafeAngle) { // makes sure that joint is in permissible range
          if (motorArray[homingMotor]->homingPass == 0) { // i can't see how this would ever be true?
#ifdef DEBUG_HOMING
            UART_PORT.print("motor "); UART_PORT.print(homingMotor + 1);
            UART_PORT.println(" homing 1 done and at safe angle");
#endif
          }
          // will only home outwards if it's double ended homing, otherwise it moves on to the next motor
          if ( (motorArray[homingMotor]->homingType == DOUBLE_ENDED_HOMING) && (motorArray[homingMotor]->homingPass == 1) ) {
#ifdef DEBUG_HOMING
            UART_PORT.print("homing motor "); UART_PORT.print(homingMotor + 1);
            UART_PORT.println(" outwards");
#endif
            motorArray[homingMotor]->homeMotor('o'); // start homing motor outwards
          }
          else { // done finding angle limits, moving to home position and then next motor time
#ifdef DEBUG_HOMING
            UART_PORT.print("motor "); UART_PORT.print(homingMotor + 1);
            UART_PORT.println(" homing complete. now to move to angle");
#endif
          /*
          motor 3 homed to -80.5 and then didnt jump out of homing sequence??
          maybe because it was supposed to be more like -79.5 since the soft angle is -80?
          I think it's cause motor 3 was a stepper and I forgot to set atSafeAngle to true in the stepper code...
          */
            if(! (motorArray[homingMotor]->startedZeroing) ){
                motorArray[homingMotor]->forceToAngle(0.0);
                motorArray[homingMotor]->startedZeroing = true;
              }
            else {
              float angle = motorArray[homingMotor]->getSoftwareAngle();
              float tolerance = motorArray[homingMotor]->pidController.getJointAngleTolerance();
              if(fabs(angle) < tolerance*2){ // within small enough angle range to move on to next motor
                motorArray[homingMotor]->homingPass = 0; // reset this for next time homing is requested
                homingMotor++; // move on to the next motor
                motorArray[homingMotor]->startedZeroing = false;
              }
              else { // not done going to zero, not doing anything
                ;
              }
            }
          }
        }
      }
      else { // not done homing the motor, will not do anything special
        ;
      }
    }
    else { // done homing all the motors
#ifdef DEBUG_HOMING
      UART_PORT.println("all motors done homing, reinitializing motor timers");
#endif
      isHoming = false;
      // this would be a good place to call the goToNeutral function or whatever
      //motorArray[homingMotor]->forceToAngle(motorArray[homingMotor]->neutralAngle);
    }
  }

  /* message parsing functionality */
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
    UART_PORT.print("GOT: "); UART_PORT.println(serialBuffer); // send back what was received
#endif
    Parser.parseCommand(motorCommand, serialBuffer); // read serialBuffer and stuff the data into motorCommand
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    msgIsValid = Parser.verifCommand(motorCommand); // verify the data to make sure it's valid
  }
#endif
  if (msgReceived) {
    if (msgIsValid) {
      if (motorCommand.pingCommand) { // respond to ping
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("pong");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("pong");
#endif
      }
      if (motorCommand.whoCommand) { // respond to ping
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("arm");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("arm");
#endif
      }
      else if (motorCommand.stopAllMotors) { // emergency stop takes precedence
        for (int i = 0; i < NUM_MOTORS; i++) {
          motorArray[i]->stopRotation();
          motorArray[i]->stopHoming();
        }
        // the following variables are global rather than belonging to a class so must be dealt with separately
        // i suppose i could package a bunch of this into a function called stopHoming
        isHoming = false;
        homingMotor = NUM_MOTORS;
        for (int i = 0; i < NUM_MOTORS; i++) {
          motorsToHome[i] = false;
        }
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("all motors stopped because of emergency stop");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("all motors stopped because of emergency stop");
#endif
      }
      else if (!isHoming) { // ignore anything besides pings or emergency stop if homing
        if (motorCommand.homeAllMotors || motorCommand.homeCommand) { // initialize homing procedure
          if (motorCommand.homeAllMotors) {
            for (int i = 0; i < NUM_MOTORS; i++) {
              if (motorArray[i]->hasLimitSwitches) {
                if (motorCommand.homingStyle == DOUBLE_ENDED_HOMING){
                  motorArray[i]->homingType = DOUBLE_ENDED_HOMING;
                }
                motorsToHome[i] = true;
#ifdef DEBUG_MAIN
                UART_PORT.print("Motor ");UART_PORT.print(i+1);UART_PORT.println(" to be homed.");
#endif
              }
            }
          }
          else if (motorCommand.homeCommand) {
            if (motorArray[motorCommand.whichMotor]->hasLimitSwitches) {
              if (motorCommand.homingStyle == DOUBLE_ENDED_HOMING){
                motorArray[motorCommand.whichMotor]->homingType = DOUBLE_ENDED_HOMING;
              }
              motorsToHome[motorCommand.whichMotor] = true;
#ifdef DEBUG_MAIN
              UART_PORT.print("Motor ");UART_PORT.print(motorCommand.whichMotor+1);UART_PORT.println(" to be homed.");
#endif
            }
          }
          isHoming = true;
          homingMotor = 0;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.println("initializing homing command");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
          nh.loginfo("initializing homing command");
#endif
        }
        else if (motorCommand.resetAllMotors) { // emergency stop takes precedence
          for (int i = 0; i < NUM_MOTORS; i++) {
            motorArray[i]->setSoftwareAngle(0.0);
          }
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.println("all motor angle values reset");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
          nh.loginfo("all motor angle values reset");
#endif
        }
        else { // following cases are for commands to specific motors
          if (motorCommand.stopSingleMotor) { // stopping a single motor takes precedence
            motorArray[motorCommand.whichMotor - 1]->stopRotation();
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("stopped motor "); UART_PORT.println(motorCommand.whichMotor);
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
          else if (motorCommand.gearCommand) { // set gear ratio for appropriate motor
            motorArray[motorCommand.whichMotor - 1]->setGearRatio(motorCommand.gearRatioVal);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has a new gear ratio of "); UART_PORT.println(motorCommand.gearRatioVal);
#endif
          }
          else if (motorCommand.openLoopGainCommand) { // set open loop gain for appropriate motor
            motorArray[motorCommand.whichMotor - 1]->setOpenLoopGain(motorCommand.openLoopGain);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has a new open loop gain of "); UART_PORT.println(motorCommand.openLoopGain);
#endif
          }
          else if (motorCommand.speedCommand) { // set speed for appropriate motor
            motorArray[motorCommand.whichMotor - 1]->setMotorSpeed(motorCommand.motorSpeed);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has a new speed of "); UART_PORT.println(motorCommand.motorSpeed);
#endif
          }
          else if (motorCommand.loopCommand) { // set loop states for appropriate motor
            if (motorCommand.loopState == OPEN_LOOP) {
              motorArray[motorCommand.whichMotor - 1]->isOpenLoop = true;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("motor "); UART_PORT.print(motorCommand.whichMotor); UART_PORT.println(" is now in open loop");
#endif
            }
            else if (motorCommand.loopState == CLOSED_LOOP) {
              if (motorArray[motorCommand.whichMotor - 1]->hasEncoder) {
                motorArray[motorCommand.whichMotor - 1]->isOpenLoop = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                UART_PORT.print("motor "); UART_PORT.print(motorCommand.whichMotor); UART_PORT.println(" is now in closed loop");
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
              motorArray[motorCommand.whichMotor - 1]->setSoftwareAngle(0.0);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("reset angle value of motor "); UART_PORT.println(motorCommand.whichMotor);
#endif
            }
            else if (motorCommand.resetJointPosition) {
              motorArray[motorCommand.whichMotor - 1]->forceToAngle(0.0);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("reset joint position (to 0 degrees) of motor "); UART_PORT.println(motorCommand.whichMotor);
#endif
            }
          }
          else if (motorCommand.switchDir) { // change the direction modifier to swap rotation direction in the case of backwards wiring
            motorArray[motorCommand.whichMotor - 1] -> switchDirectionLogic();
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            int dir = motorArray[motorCommand.whichMotor - 1]->getDirectionLogic();
            UART_PORT.print("direction modifier is now "); UART_PORT.println(dir);
#endif
          }
          else if (motorCommand.budgeCommand) { // make motors move until the command isn't sent anymore
            for (int i = 0; i < NUM_MOTORS; i++) {
              if (motorCommand.motorsToMove[i]) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                UART_PORT.print("motor "); UART_PORT.print(i + 1); UART_PORT.print(" desired direction is: "); UART_PORT.println(motorCommand.directionsToMove[i]);
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
                motorArray[i]->budge(motorCommand.directionsToMove[i]);
              }
            }
          }
          else if (motorCommand.multiMove) { // make motors move simultaneously
            for (int i = 0; i < NUM_MOTORS; i++) {
              if (motorCommand.motorsToMove[i]) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#ifdef DEBUG_MAIN
                UART_PORT.print("motor "); UART_PORT.print(i + 1); UART_PORT.print(" desired angle (degrees) is: "); UART_PORT.println(motorCommand.anglesToReach[i]);
#endif
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
                if (!(motorArray[i] -> withinJointAngleLimits(motorCommand.anglesToReach[i]))) {
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
                else {
                  if (motorArray[i]->setDesiredAngle(motorCommand.anglesToReach[i])) { // this method returns true if the command is within joint angle limits
                    motorArray[i]->goToCommandedAngle();
                  }
                }
              }
            }
          }
        }
      }
      else { // alert the user that the arm is homing so ignoring certain commands
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("arm is homing! ignoring all commands besides ping or stop");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("arm is homing! ignoring all commands besides ping or stop");
#endif
      }
    }
    else { // alert the user that it's a bad command
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.println("$E,Error: bad motor command");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
      nh.logerror("error: bad motor command");
#endif
    }
    /*
    // blinkled stuff
    msgCheck = true; //Setting message check value to TRUE as a message is received
    if (msgIsValid == true){
      msgState = true;
    }
    else{
      msgState = false;
    }
    */
  }
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) { // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }
  
  /* heartbeat code blinkled stuff */
  /*
  if(msgCheck == true){
    if(msgState == true){
      msgCheck = Blink(goodBlinkInterval, goodBlinkCounter);
    }
    else {
      msgCheck = Blink(badBlinkInterval, badBlinkCounter);
    }
  }
  else {
    heartbeat();
  }
*/  
}


void printMotorAngles(void) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  UART_PORT.print("Motor Angles: ");
  for (int i = 0; i < NUM_MOTORS; i++) {
    UART_PORT.print(motorArray[i]->getSoftwareAngle());
    if (i < NUM_MOTORS - 1) {
      UART_PORT.print(", ");
    }
    else {
      UART_PORT.println("");
    }
  }
#elif defined(DEBUG_MODE) || defined(USER_MODE)
  float angles[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    angles[i] = motorArray[i]->getSoftwareAngle();
    angleMessages[i].position = &(angles[i]);
    //*(angleMessages[i].position) = motorArray[i]->getSoftwareAngle();
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

void initComms(void) {
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
}

void initEncoders(void) {
  // each motor with an encoder needs to attach the encoder and 2 interrupts
  // this function also sets pid parameters
#ifdef M1_ENCODER_PORT
  motor1.attachEncoder(M1_ENCODER_A, M1_ENCODER_B, M1_ENCODER_PORT, M1_ENCODER_SHIFT, M1_ENCODER_RESOLUTION);
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
}

void initLimitSwitches(void) {
  // c for clockwise/counterclockwise, f for flexion/extension, g for gripper
  //motor1.attachLimitSwitches(REVOLUTE_SWITCH, M1_LIMIT_SW_CW, M1_LIMIT_SW_CCW);
  motor2.attachLimitSwitches(FLEXION_SWITCH, M2_LIMIT_SW_FLEX, M2_LIMIT_SW_EXTEND);
  motor3.attachLimitSwitches(FLEXION_SWITCH, M3_LIMIT_SW_FLEX, M3_LIMIT_SW_EXTEND);
  motor4.attachLimitSwitches(FLEXION_SWITCH, M4_LIMIT_SW_FLEX, M4_LIMIT_SW_EXTEND);
  //motor5.attachLimitSwitches(REVOLUTE_SWITCH, M5_LIMIT_SW_CW, M5_LIMIT_SW_CCW);
  //motor6.attachLimitSwitches(GRIPPER_SWITCH, 0, M6_LIMIT_SW_EXTEND); // only checks for gripper opening
  //attachInterrupt(motor1.limSwitchCw, m1CwISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor1.limSwitchCcw, m1CcwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchFlex, m2FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchExtend, m2ExtendISR, LIM_SWITCH_DIR);
  attachInterrupt(motor3.limSwitchFlex, m3FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor3.limSwitchExtend, m3ExtendISR, LIM_SWITCH_DIR);
  attachInterrupt(motor4.limSwitchFlex, m4FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor4.limSwitchExtend, m4ExtendISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor5.limSwitchCw, m5CwISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor5.limSwitchCcw, m5CcwISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor6.limSwitchOpen, m6OpenISR, LIM_SWITCH_DIR);

  // set motor joint angle limits
  motor1.setAngleLimits(M1_MIN_HARD_ANGLE, M1_MAX_HARD_ANGLE, M1_MIN_SOFT_ANGLE, M1_MAX_SOFT_ANGLE);
  motor2.setAngleLimits(M2_MIN_HARD_ANGLE, M2_MAX_HARD_ANGLE, M2_MIN_SOFT_ANGLE, M2_MAX_SOFT_ANGLE);
  motor3.setAngleLimits(M3_MIN_HARD_ANGLE, M3_MAX_HARD_ANGLE, M3_MIN_SOFT_ANGLE, M3_MAX_SOFT_ANGLE);
  motor4.setAngleLimits(M4_MIN_HARD_ANGLE, M4_MAX_HARD_ANGLE, M4_MIN_SOFT_ANGLE, M4_MAX_SOFT_ANGLE);
  // motor5.setAngleLimits(M5_MIN_HARD_ANGLE, M5_MAX_HARD_ANGLE, M5_MIN_SOFT_ANGLE, M5_MAX_SOFT_ANGLE); // this joint should be able to spin freely
  motor6.setAngleLimits(M6_MIN_HARD_ANGLE, M6_MAX_HARD_ANGLE, M6_MIN_SOFT_ANGLE, M6_MAX_SOFT_ANGLE);

  // set motor shaft angle tolerances
  motor1.pidController.setJointAngleTolerance(0.1);//2.0 * motor1.gearRatioReciprocal); // randomly chosen for dc
  motor2.pidController.setJointAngleTolerance(0.1);//2.0 * motor2.gearRatioReciprocal);
  motor3.pidController.setJointAngleTolerance(0.1);//2.0 * 2 * motor3.gearRatioReciprocal);
  motor4.pidController.setJointAngleTolerance(0.1);//1.8 * 2 * motor4.gearRatioReciprocal); // 1.8 is the min stepper resolution so I gave it +/- tolerance
  motor5.pidController.setJointAngleTolerance(0.1);//2.0 * motor5.gearRatioReciprocal); // randomly chosen for servo
  motor6.pidController.setJointAngleTolerance(0.1);//2.0 * motor6.gearRatioReciprocal);
}

/*! Sets pidController output limits for each motor, then sets openLoopSpeed for each motor,
 * and finally, sets openLoopGain for any non-stepper motor.
 *  
 * \todo stepper doesn't have speed the way servos and dcs do??????
 * this is because stepper calculates speed using durations, but this means
 * openLoopGain and openLoopSpeed mean nothing for it???
*/
void initSpeedParams(void) {
  // set max and min speeds (in percentage)
  // Abtin thinks 50% should be a hard limit that can't be modified this easily
  motor1.setMotorSpeed(50); // needs to be tuned
  motor2.setMotorSpeed(30); // needs to be tuned
  motor3.setMotorSpeed(50); // needs to be tuned
  motor4.setMotorSpeed(90); // needs to be tuned?
  motor5.setMotorSpeed(50); // probably done tuning
  motor6.setMotorSpeed(50); // probably done tuning

  // set pid slowest speed before it cuts power, to avoid noise and energy drain
  motor1.pidController.setSlowestSpeed(5.0); // needs to be tuned
  motor2.pidController.setSlowestSpeed(5.0); // needs to be tuned
  motor3.pidController.setSlowestSpeed(5.0); // needs to be tuned
  //motor4.pidController.setSlowestSpeed(5.0); //honestly it doesn't really make sense for steppers? or does it
  //motor5.pidController.setSlowestSpeed(5.0); // servos have no closed loop
  //motor6.pidController.setSlowestSpeed(5.0); // servos have no closed loop

  // set open loop parameters. By default the motors are open loop,
  // have constant velocity profiles (no ramping), operate at 50% max speed,
  // and the gains should vary based on which motor it is

  // open loop gain is only for time-based open loop control
  motor1.setOpenLoopGain(0.001); // needs to be tuned
  motor2.setOpenLoopGain(0.0025); // needs to be tuned
  motor3.setOpenLoopGain(0.00455); // more or less tuned
  motor5.setOpenLoopGain(0.32); // for 5V, probably done tuning
  motor6.setOpenLoopGain(0.35); // for 5V, probably done tuning
}

void initMotorTimers(void) {
  dcTimer.begin(dcInterrupt, DC_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  dcTimer.priority(MOTOR_NVIC_PRIORITY);
  m4StepperTimer.begin(m4StepperInterrupt, STEPPER_PID_PERIOD); // 1000ms
  m4StepperTimer.priority(MOTOR_NVIC_PRIORITY);
  servoTimer.begin(servoInterrupt, SERVO_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  servoTimer.priority(MOTOR_NVIC_PRIORITY);
}

void dcInterrupt(void) {
  motor1.motorTimerInterrupt();
  motor2.motorTimerInterrupt();
  motor3.motorTimerInterrupt();

}
void m4StepperInterrupt(void) {
  motor4.motorTimerInterrupt(m4StepperTimer);
}
void servoInterrupt(void) {
  motor5.motorTimerInterrupt();
  motor6.motorTimerInterrupt();
}

#ifdef M1_ENCODER_PORT
void m1_encoder_interrupt(void) {
  /*! encoder states are 4 bit values. Top 2 bits are the previous states
   * of encoder channels A and B, bottom 2 are current states.
  */
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2; //!< shift current state into previous state
  /*! put the 2 relevant pin states from the relevant gpio port into memory, clearing the irrelevant bits
   * this is done by 1) grabbing the input register of the port,
   * 2) shifting it until the relevant bits are in the lowest state,
   * and 3) clearing all the bits higher than the lowest 2
   * next, place said (current) pin states into the bottom 2 bits of oldEncoderState
   */
  oldEncoderState |= ((M1_ENCODER_PORT >> M1_ENCODER_SHIFT) & 0x03);
  /*! the encoderStates[] array corresponds to the correct direction
   * for a specific set of prev and current encoder states.
   * The & operation ensures that anything above the lowest 4 bits
   * is cleared before accessing the array.
  */
  motor1.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 1 "); UART_PORT.println(motor1.encoderCount);
#endif
}
/*
  //! use this version if only one encoder channel is working
  void m1_encoder_interrupt(void) {
  motor1.encoderCount += motor1.rotationDirection * 2;
  #ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 1 ");UART_PORT.println(motor1.encoderCount);
  #endif
  }
*/
#endif
#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
#ifdef DEBUG_ENCODERS
  UART_PORT.print("motor 2 "); UART_PORT.println(motor2.encoderCount);
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
  UART_PORT.print("motor 3 "); UART_PORT.println(motor3.encoderCount);
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
  UART_PORT.print("motor 4 "); UART_PORT.println(motor4.encoderCount);
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
  UART_PORT.print("motor 5 "); UART_PORT.println(motor5.encoderCount);
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
  UART_PORT.print("motor 6 "); UART_PORT.println(motor6.encoderCount);
#endif
}
#endif

void m1CwISR(void) {
  // if the switch wasn't previously triggered then it starts a counter
  if (!(motor1.triggered) && motor1.triggerState == 0) {
    motor1.triggered = true;
    motor1.sinceTrigger = 0;
  }
  // grab the state of the pin for the cw switch
  int pinState = (M1_LIMIT_SW_CW_PORT >> M1_LIMIT_SW_CW_SHIFT ) & 1;
  // since we care about falling edges, when it reads 0 it's a hit
  // a hit is +1 or -1 depending on which switch was hit
  if (pinState == 0) {
    motor1.triggerState = CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("clockwise");
#endif
  }
  // if it's not a hit we set it to 0
  else {
    motor1.triggerState = 0;
  }
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
  void m5CwISR(void) {
    ;
  }
  void m5CcwISR(void) {
	  ;
  }
  void m6OpenISR(void) {
    ;
  }
*/

/*LED blink when message received */
/*When a good message is received, ledInterval = 250, maxBlinks = 4*/
/*When a bad message is received, ledInterval = 100, maxBlinks = 12*/
/*
bool Blink(const int ledInterval, int maxBlinks){ 
  static bool complete = false;
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= ledInterval){ 
    previousMillis = currentMillis;
    if(ledState == LOW){
      ledState = HIGH;
    } else{
      ledState = LOW;
    }
    digitalWrite(led, ledState);
    Serial.println(ledState);
    blinkCounter++;
  }
  if(blinkCounter == maxBlinks){ 
    complete = true;
    blinkCounter = 0;
  }
  return complete;
}

void heartbeat(){
  
}
*/
