/*
  This is the main sketch which defines the control logic of the robotic arm of
  the Space Concordia Division, which runs on a Teensy 3.6 that communicates
  with an Odroid XU4.

  The code can be compiled for serial communication over usb (if connected to a
  standard computer), or for serial communication over TX/RX pins (if connected
  to the Odroid). In the latter case, communication can be done either directly
  with Serial1, or with ROSserial, if integration in the ROS network is desired.
  Currently, this code is built for the control of six motors: three DC motors,
  one stepper motor, and two continuous rotation servos. Several helper classes
  were written to abstract away the complexities of controlling different types
  of motors and communicating with a master device.

  The code allows position control for all six of the arm's joints. It also
  has a homing routine which depends on limit switches and sets the
  0 degrees position for each joint. The code also allows to stop motors at
  any point in time. Open loop control can be performed without the use of
  encoders but results in imprecise and jerky control. Use of ramping allows
  for less jerky control. The best control is closed loop control, which uses
  encoders for smooth speed profiles. Many parameters can be adjusted with the
  appropriate command. The Teensy also has a software reboot command.

  This code began development sometime in July 2018 and is still being
  updated as of April 24 2019.
*/

#include "Includes.h"

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

/* blink variables */
bool msgCheck = false; //!< If a message was received, while this remains true, the MCU will blink
enum blinkTypes {HEARTBEAT, GOOD_BLINK, BAD_BLINK}; //!< blink style depends on what's going on
int blinkType = HEARTBEAT; //!< by default it should be the heartbeat. Will behave differently if message is received
bool startBlinking = false; //!< if true, teensy blinks as response to a message
int blinkCount = 0; //!< when it reaches max it goes back to heartbeat

// i took the ROS stuff from here and stuck it into ideas.h

/* motors */
//! quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states
const int encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// instantiate motor objects here:
DcMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO);
DcMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO);
DcMotor motor3(M3_DIR_PIN, M3_PWM_PIN, M3_GEAR_RATIO);
DcMotor motor4(M4_DIR_PIN, M4_PWM_PIN, M4_GEAR_RATIO);
ServoMotor motor5(M5_PWM_PIN, M5_GEAR_RATIO);
ServoMotor motor6(M6_PWM_PIN, M6_GEAR_RATIO);

// motor array prep work: making pointers to motor objects
DcMotor *m1 = &motor1; DcMotor *m2 = &motor2; DcMotor *m3 = &motor3; DcMotor *m4 = &motor4;
ServoMotor *m5 = &motor5; ServoMotor *m6 = &motor6;
RobotMotor *motorArray[] = {m1, m2, m3, m4, m5, m6}; //!< I can use this instead of switch/case statements by doing motorArray[motornumber]->attribute
// instantiate timers here:
IntervalTimer dcTimer; // motors 1,2,3&4
IntervalTimer servoTimer; // motors 5&6

// these are a nicer way of timing events than using millis()
elapsedMillis sinceAnglePrint; //!< how long since last time angle data was sent
elapsedMillis sinceBlink; //!< how long since previous blink occurred

// homing variables
bool isHoming = false; //!< true while arm is homing, false otherwise
int homingMotor = -1; //! initialize to a value that's invalid so it'll be ignored. Used in main loop to remember which motors need to be homed since commandInfo is reset each loop iteration
//! used in main loop to remember which motors need to be homed since commandInfo is reset each loop iteration
bool motorsToHome[] = {false, false, false, false, false, false};
/* function declarations */
void initComms(void); //!< start up serial or usb communication
void initEncoders(void); //!< attach encoder interrupts and setup pid gains
void initLimitSwitches(void); //!< setup angle limits and attach limit switch interrupts
void initSpeedParams(void); //!< setup open and closed loop speed parameters
void initMotorTimers(void); //!< start the timers which control the motors

void printMotorAngles(void); //!< sends all motor angles over serial
void clearBlinkState(void); //!< sets some blink variables to zero
void blinkLED(void); //!< blinks LED based on global variables
void respondToLimitSwitches(void); //!< move motor back into software angle range. This function behaves differently each loop
void homeArmMotors(void); //!< arm homing routine. This function behaves differently each loop
void rebootTeensy(void); //!< reboots the teensy using the watchdog timer
void kickDog(void); //!< resets the watchdog timer. If the teensy is stuck in a loop and this is not called, teensy resets.
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
void m6FlexISR(void);
void m6ExtendISR(void);

// declare timer interrupt service routines, where the motors actually get controlled
void dcInterrupt(void); //!< manages motors 1-4
void servoInterrupt(void); //!< manages motors 5&6

/*! \brief Teensy setup. Calls many init functions to prep comms and motors.

   \todo do housekeeping stuff
   \todo Implement error checking, clean up code, comment code, make sure ISR variables are volatile
   \todo Make sure the math calculations are written correctly and calculate as quickly as possible.
   Floating point math doesn't seem bad, but at worst, convert float to int
   before motor control and do int math inside interrupts.
   \todo (Nick) Finish implementing/integrating heartbeat and watchdog interrupt
   \todo Figure out where to disable interrupts so that I don't read a value while it's being modified
   \todo Determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
   \todo Issue: interrupt functions must be defined outside of classes...
   \todo Confirm all the pins will work with interrupts and not stepping on each other.
   Do I send 3.3v to all interrupt pins simultaneously to test thing?
   \todo Deal with overflow of encoderCount.. does it ever reach max value?
   \todo What do I do for angles over 360? Do I keep counting up?
   Do I keep count of how many rotations I've done?
   \todo Different types of ramping profiles - trapezoid vs quintic polynomial?
   Ramping of stepper should be linear, higher level ramping should occur in gui.
   The base station/odroid should be in charge of ramping up angles and the teensy
   should just go to them? Perhaps if a large angle is requested there should
   still be a way to stop it though.
   \todo Determine whether it's worth it to use the built in quadrature decoders.
   Quadrature on ftm1,2: pins 3/4,29/30: cant use for pwm anymore.
   Quadrature on tpm1,2: pins 16/17, (tpm2 not implemented in teensy?).
*/
void setup() {
  pinSetup();
  initComms();
  initEncoders();
  initLimitSwitches(); //!< \todo setJointAngleTolerance in here might need to be adjusted when gear ratio is adjusted!!! check other dependencies too!!!
  initSpeedParams();
  motor3.switchDirectionLogic(); // motor is wired backwards? replaced with dc, needs new test
  motor4.switchDirectionLogic(); // motor is wired backwards? replaced with dc, needs new test
  initMotorTimers();

  // reset the elapsedMillis variables so that they're fresh upon entering the loop()
  sinceAnglePrint = 0;
}

/*! \brief Main code which loops forever. Parses commands, prints motor angles and blinks the builtin LED.

   \todo There should be a check so that if the motor is moving away from the goal position
   or has been moving for a while without reaching the expected angle, it stops...
   like a timeout.
   \todo What happens if a new command tells the motor to turn in opposite direction? Abrupt changes are bad.
   \todo If the stepper is trying to turn but hasn't gotten anywhere, there should be
   a check in the microcontroller that there's an issue (there can also be a check in the gui)
   \todo Check to see if any global variables can be turned into static variables inside loop()
   \todo I noticed that sending a new move command while motors are moving messes with open loop calculations?
*/
void loop() {
  respondToLimitSwitches(); // limit switch checks occur before listening for commands. This function behaves differently each loop
  if (isHoming) { // not done homing the motors
    homeArmMotors(); // Homing functionality ignores most message types. This function behaves differently each loop
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
    UART_PORT.print("ARM GOT: "); UART_PORT.println(serialBuffer); // send back what was received
#endif
    Parser.parseCommand(motorCommand, serialBuffer); // read serialBuffer and stuff the data into motorCommand
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    msgIsValid = Parser.verifCommand(motorCommand); // verify the data to make sure it's valid
  }
#endif
  if (msgReceived) {
    startBlinking = true;
    if (msgIsValid) {
      blinkType = GOOD_BLINK; // alert the blink check that it was a good message
      if (motorCommand.pingCommand) { // respond to ping
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("ARM pong");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("pong");
#endif
      }
      else if (motorCommand.whoCommand) { // respond to ping
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("ARM arm");
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
        homingMotor = NUM_MOTORS - 1;
        for (int i = 0; i < NUM_MOTORS; i++) {
          motorsToHome[i] = false;
        }
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("ARM all motors stopped because of emergency stop");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("all motors stopped because of emergency stop");
#endif
      }
      else if (motorCommand.rebootCommand) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("ARM rebooting arm teensy... hang on a sec");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("rebooting arm teensy... hang on a sec");
#endif
        rebootTeensy();
      }
      else if (!isHoming) { // ignore anything besides pings or emergency stop if homing
        if (motorCommand.homeAllMotors || motorCommand.homeCommand) { // initialize homing procedure
          if (motorCommand.homeAllMotors) {
            for (int i = NUM_MOTORS - 1; i >= 0; i--) { // start with last motor and work inwards
              if (motorArray[i]->hasLimitSwitches) {
                if (motorCommand.homingStyle == DOUBLE_ENDED_HOMING) {
                  motorArray[i]->homingType = DOUBLE_ENDED_HOMING;
                }
                motorsToHome[i] = true;
#ifdef DEBUG_MAIN
                UART_PORT.print("ARM Motor "); UART_PORT.print(i + 1); UART_PORT.println(" to be homed.");
#endif
              }
            }
            homingMotor = NUM_MOTORS - 1;
          }
          else if (motorCommand.homeCommand) {
            if (motorArray[motorCommand.whichMotor - 1]->hasLimitSwitches) {
              if (motorCommand.homingStyle == DOUBLE_ENDED_HOMING) {
                motorArray[motorCommand.whichMotor - 1]->homingType = DOUBLE_ENDED_HOMING;
              }
              motorsToHome[motorCommand.whichMotor - 1] = true;
#ifdef DEBUG_MAIN
              UART_PORT.print("ARM Motor "); UART_PORT.print(motorCommand.whichMotor); UART_PORT.println(" to be homed.");
#endif
            }
            homingMotor = motorCommand.whichMotor - 1;
          }
          isHoming = true;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.print("ARM initializing homing command, starting with motor ");
          UART_PORT.println(homingMotor + 1);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
          nh.loginfo("initializing homing command");
#endif
        }
        else if (motorCommand.resetAllMotors) { // reset software angles of all motors
          for (int i = 0; i < NUM_MOTORS; i++) {
            motorArray[i]->setSoftwareAngle(0.0);
          }
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
          UART_PORT.println("ARM all motor angle values reset");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
          nh.loginfo("all motor angle values reset");
#endif
        }
        else if (motorCommand.armSpeedCommand) {
          float factor = motorCommand.armSpeedMultiplier;
          if (factor > 0) {
            for (int i = 0; i < NUM_MOTORS; i++) {
              float newSpeed = ( motorArray[i]->getMotorSpeed() ) * factor;
              if (newSpeed >= 100) {
#ifdef DEBUG_MAIN
                UART_PORT.print("ARM $A,Alert: multiplier is too big, motor ");
                UART_PORT.print(i + 1);
                UART_PORT.println(" speed is saturating at 100%");
#endif
                newSpeed = 100;
              }
#ifdef DEBUG_MAIN
              UART_PORT.print("ARM $S,Success: Motor "); UART_PORT.print(i + 1);
              UART_PORT.print(" speed is now "); UART_PORT.println(newSpeed);
#endif
              motorArray[i]->setMotorSpeed(newSpeed);
            }
          }
          else {
#ifdef DEBUG_MAIN
            UART_PORT.println("ARM $E,Error: invalid multiplier value");
#endif
          }
        }
        else { // following cases are for commands to specific motors
          if (motorCommand.stopSingleMotor) { // stopping a single motor takes precedence
            motorArray[motorCommand.whichMotor - 1]->stopRotation();
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("ARM stopped motor "); UART_PORT.println(motorCommand.whichMotor);
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
            UART_PORT.print("ARM motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has a new gear ratio of "); UART_PORT.println(motorCommand.gearRatioVal);
#endif
          }
          else if (motorCommand.openLoopGainCommand) { // set open loop gain for appropriate motor
            motorArray[motorCommand.whichMotor - 1]->setOpenLoopGain(motorCommand.openLoopGain);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("ARM motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has a new open loop gain of "); UART_PORT.println(motorCommand.openLoopGain);
#endif
          }
          else if (motorCommand.pidCommand) { // set open loop gain for appropriate motor
            motorArray[motorCommand.whichMotor - 1]->pidController.setGainConstants(motorCommand.kp, motorCommand.ki, motorCommand.kd);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("ARM motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has new pid gains. kp: ");
            UART_PORT.print(motorCommand.kp); UART_PORT.print(" ki: ");
            UART_PORT.print(motorCommand.ki); UART_PORT.print(" kd: ");
            UART_PORT.println(motorCommand.kd);
#endif
          }
          else if (motorCommand.motorSpeedCommand) { // set speed for appropriate motor
            motorArray[motorCommand.whichMotor - 1]->setMotorSpeed(motorCommand.motorSpeed);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("ARM motor "); UART_PORT.print(motorCommand.whichMotor);
            UART_PORT.print(" has a new speed of "); UART_PORT.println(motorCommand.motorSpeed);
#endif
          }
          else if (motorCommand.loopCommand) { // set loop states for appropriate motor
            if (motorCommand.loopState == OPEN_LOOP) {
              motorArray[motorCommand.whichMotor - 1]->isOpenLoop = true;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
              UART_PORT.print("ARM motor "); UART_PORT.print(motorCommand.whichMotor); UART_PORT.println(" is now in open loop");
#endif
            }
            else if (motorCommand.loopState == CLOSED_LOOP) {
              if (motorArray[motorCommand.whichMotor - 1]->hasEncoder) {
                motorArray[motorCommand.whichMotor - 1]->isOpenLoop = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                UART_PORT.print("ARM motor "); UART_PORT.print(motorCommand.whichMotor); UART_PORT.println(" is now in closed loop");
#endif
              }
              else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                UART_PORT.println("ARM $E,Alert: cannot use closed loop if motor has no encoder.");
#endif
              }
            }
          }
          else if (motorCommand.resetSingleMotor) { // reset the motor angle's variable
            motorArray[motorCommand.whichMotor - 1]->setSoftwareAngle(0.0);
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("ARM reset angle value of motor "); UART_PORT.println(motorCommand.whichMotor);
#endif
          }
          else if (motorCommand.switchDir) { // change the direction modifier to swap rotation direction in the case of backwards wiring
            motorArray[motorCommand.whichMotor - 1] -> switchDirectionLogic();
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            int dir = motorArray[motorCommand.whichMotor - 1]->getDirectionLogic();
            UART_PORT.print("ARM direction modifier is now "); UART_PORT.println(dir);
#endif
          }
          else if (motorCommand.budgeCommand) { // make motors move until the command isn't sent anymore
            for (int i = 0; i < NUM_MOTORS; i++) {
              if (motorCommand.motorsToMove[i]) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
                UART_PORT.print("ARM motor "); UART_PORT.print(i + 1); UART_PORT.print(" desired direction is: "); UART_PORT.println(motorCommand.directionsToMove[i]);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
                // this is SUPER DUPER GROSS
                int tempVal = i + 1;
                String infoMessage = "ARM motor " + tempVal;
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
                UART_PORT.print("ARM motor "); UART_PORT.print(i + 1); UART_PORT.print(" desired angle (degrees) is: "); UART_PORT.println(motorCommand.anglesToReach[i]);
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
                  UART_PORT.print("ARM $E,Error: requested motor ");
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
        } // end of commands to a specific motor
      } // end of commands ignored during homing
      else { // alert the user that the arm is homing so ignoring certain commands
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.println("ARM arm is homing! ignoring all commands besides ping or stop");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        nh.loginfo("arm is homing! ignoring all commands besides ping or stop");
#endif
      }
    } // end of executing valid commands
    else { // alert the user that it's a bad command
      blinkType = BAD_BLINK; // alert the blink check that it was a bad message
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.println("ARM $E,Error: bad motor command");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
      nh.logerror("error: bad motor command");
#endif
    }
  } // end of message parsing
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) { // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
    printMotorAngles();
    vbatt_read();
    sinceAnglePrint = 0; // reset the timer
  }

  if (startBlinking) { // freshen things up before changing blink type
    startBlinking = false;
    clearBlinkState();
  }
  blinkLED(); // decides how to blink based on global variables
} // end of loop

/* initialization functions */
void initComms(void) { //!< Starts up serial comms over USB or UART and uses ROSserial if the macro is defined
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
/*! Each motor with an encoder needs to attach the encoder and 2 interrupts.
   This function also sets pid parameters and joint angle tolerances.
*/
void initEncoders(void) {
#ifdef DEBUG_MAIN
  UART_PORT.println("ARM Attaching encoders to motor objects and attaching encoder interrupts, changing edge.");
#endif

  motor1.attachEncoder(M1_ENCODER_A, M1_ENCODER_B, M1_ENCODER_PORT, M1_ENCODER_SHIFT, M1_ENCODER_RESOLUTION);
  attachInterrupt(motor1.encoderPinA, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor1.encoderPinB, m1_encoder_interrupt, CHANGE);

  motor2.attachEncoder(M2_ENCODER_A, M2_ENCODER_B, M2_ENCODER_PORT, M2_ENCODER_SHIFT, M2_ENCODER_RESOLUTION);
  attachInterrupt(motor2.encoderPinA, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinB, m2_encoder_interrupt, CHANGE);

  motor3.attachEncoder(M3_ENCODER_A, M3_ENCODER_B, M3_ENCODER_PORT, M3_ENCODER_SHIFT, M3_ENCODER_RESOLUTION);
  attachInterrupt(motor3.encoderPinA, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinB, m3_encoder_interrupt, CHANGE);

  motor4.attachEncoder(M4_ENCODER_A, M4_ENCODER_B, M4_ENCODER_PORT, M4_ENCODER_SHIFT, M4_ENCODER_RESOLUTION);
  attachInterrupt(motor4.encoderPinA, m4_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinB, m4_encoder_interrupt, CHANGE);

  // set activate PIDs
  motor1.isOpenLoop = false; motor1.encoderModifier=-1;
  motor2.isOpenLoop = false;
  motor3.isOpenLoop = false;
  motor4.isOpenLoop = false; //motor4.encoderModifier=-1;

  // set pid gains
  motor1.pidController.setGainConstants(10.0, 0.0, 0.0);
  motor2.pidController.setGainConstants(10.0, 0.0, 0.0);
  motor3.pidController.setGainConstants(10.0, 0.0, 0.0);
  motor4.pidController.setGainConstants(10.0, 0.0, 0.0);

  // set motor shaft angle tolerances
  motor1.pidController.setJointAngleTolerance(0.1);//2.0 * motor1.gearRatioReciprocal); // randomly chosen for dc
  motor2.pidController.setJointAngleTolerance(0.1);//2.0 * motor2.gearRatioReciprocal);
  motor3.pidController.setJointAngleTolerance(0.1);//2.0 * motor3.gearRatioReciprocal);
  motor4.pidController.setJointAngleTolerance(0.1);//2.0 * motor4.gearRatioReciprocal);
  motor5.pidController.setJointAngleTolerance(0.1);//2.0 * motor5.gearRatioReciprocal); // randomly chosen for servo
  motor6.pidController.setJointAngleTolerance(0.1);//2.0 * motor6.gearRatioReciprocal);
}
/*! Each motor with limit switches needs to attach the limit switch and 2 ish interrupts.
    this function also sets angle limits for the joints
*/
void initLimitSwitches(void) {
#ifdef DEBUG_MAIN
  UART_PORT.println("ARM Attaching limit switches to motor objects and attaching limit switch interrupts, falling edge.");
#endif

  // c for clockwise/counterclockwise, f for flexion/extension, g for gripper (assuming only one switch)
  motor1.attachLimitSwitches(REVOLUTE_SWITCH, M1_LIMIT_SW_CW, M1_LIMIT_SW_CCW);
  motor2.attachLimitSwitches(FLEXION_SWITCH, M2_LIMIT_SW_FLEX, M2_LIMIT_SW_EXTEND);
  motor3.attachLimitSwitches(FLEXION_SWITCH, M3_LIMIT_SW_FLEX, M3_LIMIT_SW_EXTEND);
  motor4.attachLimitSwitches(FLEXION_SWITCH, M4_LIMIT_SW_FLEX, M4_LIMIT_SW_EXTEND);
  //motor5.attachLimitSwitches(REVOLUTE_SWITCH, M5_LIMIT_SW_CW, M5_LIMIT_SW_CCW);
  motor6.attachLimitSwitches(FLEXION_SWITCH, M6_LIMIT_SW_FLEX, M6_LIMIT_SW_EXTEND);

  attachInterrupt(motor1.limSwitchCw, m1CwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor1.limSwitchCcw, m1CcwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchFlex, m2FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchExtend, m2ExtendISR, LIM_SWITCH_DIR);
  attachInterrupt(motor3.limSwitchFlex, m3FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor3.limSwitchExtend, m3ExtendISR, LIM_SWITCH_DIR);
  attachInterrupt(motor4.limSwitchFlex, m4FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor4.limSwitchExtend, m4ExtendISR, LIM_SWITCH_DIR);
  //attachInterrupt(motor5.limSwitchCw, m5CwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor6.limSwitchFlex, m6FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor6.limSwitchExtend, m6ExtendISR, LIM_SWITCH_DIR);

  // set motor joint angle limits
  motor1.setAngleLimits(M1_MIN_HARD_ANGLE, M1_MAX_HARD_ANGLE, M1_MIN_SOFT_ANGLE, M1_MAX_SOFT_ANGLE);
  motor2.setAngleLimits(M2_MIN_HARD_ANGLE, M2_MAX_HARD_ANGLE, M2_MIN_SOFT_ANGLE, M2_MAX_SOFT_ANGLE);
  motor3.setAngleLimits(M3_MIN_HARD_ANGLE, M3_MAX_HARD_ANGLE, M3_MIN_SOFT_ANGLE, M3_MAX_SOFT_ANGLE);
  motor4.setAngleLimits(M4_MIN_HARD_ANGLE, M4_MAX_HARD_ANGLE, M4_MIN_SOFT_ANGLE, M4_MAX_SOFT_ANGLE);
  // motor5.setAngleLimits(M5_MIN_HARD_ANGLE, M5_MAX_HARD_ANGLE, M5_MIN_SOFT_ANGLE, M5_MAX_SOFT_ANGLE); // this joint should be able to spin freely
  motor6.setAngleLimits(M6_MIN_HARD_ANGLE, M6_MAX_HARD_ANGLE, M6_MIN_SOFT_ANGLE, M6_MAX_SOFT_ANGLE);
}
/*! Sets pidController output limits for each motor, then sets openLoopSpeed for each motor.
   Also sets openLoopGain for any non-stepper motor.

   \todo stepper doesn't have speed the way servos and dcs do??????
   this is because stepper calculates speed using durations, but this means
   openLoopGain and openLoopSpeed mean nothing for it???
*/
void initSpeedParams(void) {
  // set speed limits (in percentage). It also sets open loop gains to make sure the angle estimations are ok
  // Abtin thinks 50% should be a hard limit that can't be modified this easily
#ifdef DEBUG_MAIN
  UART_PORT.println("ARM Setting motor speeds and open loop gains.");
#endif

  motor1.setMotorSpeed(50); // 60 rpm
  motor2.setMotorSpeed(42); // 32 rpm
  motor3.setMotorSpeed(65); // 45 rpm
  motor4.setMotorSpeed(30); // 60 rpm
  motor5.setMotorSpeed(100);
  motor6.setMotorSpeed(100);

  // set pid slowest speed before it cuts power, to avoid noise and energy drain
  motor1.pidController.setSlowestSpeed(5.0); // needs to be tuned
  motor2.pidController.setSlowestSpeed(10.0); // needs to be tuned
  motor3.pidController.setSlowestSpeed(5.0); // needs to be tuned
  motor4.pidController.setSlowestSpeed(5.0); // needs to be tuned
  //motor5.pidController.setSlowestSpeed(5.0); // servos have no closed loop
  //motor6.pidController.setSlowestSpeed(5.0); // servos have no closed loop

  // set open loop parameters. By default the motors are open loop,
  // have constant velocity profiles (no ramping), operate at 50% max speed,
  // and the gains should vary based on which motor it is

  // open loop gain is only for time-based open loop control
  motor1.setOpenLoopGain(0.001); // needs to be tuned
  motor2.setOpenLoopGain(0.0012); // needs to be tuned
  motor3.setOpenLoopGain(0.004); // more or less tuned
  motor4.setOpenLoopGain(0.04); // needs to be tuned
  motor5.setOpenLoopGain(0.32);
  motor6.setOpenLoopGain(0.25);
}
//! Attaches interrupt functions to motor timers. Also sets interrupt priorities.
void initMotorTimers(void) {
#ifdef DEBUG_MAIN
  UART_PORT.println("ARM Starting up motor timer interrupts.");
#endif

  dcTimer.begin(dcInterrupt, DC_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  dcTimer.priority(MOTOR_NVIC_PRIORITY);
  servoTimer.begin(servoInterrupt, SERVO_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  servoTimer.priority(MOTOR_NVIC_PRIORITY);
}

/* functions which apply to all motors or to the teensy in general */
void printMotorAngles(void) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  UART_PORT.print("ARM Motor Angles: ");
  for (int i = 0; i < NUM_MOTORS; i++) {
    motorArray[i]->calcCurrentAngle();
    UART_PORT.print(motorArray[i]->getSoftwareAngle());
    if (i < NUM_MOTORS - 1) {
      UART_PORT.print(", ");
    }
    else {
      UART_PORT.println("");
    }
  }
  //UART_PORT.println(motor1.encoderCount);
  //UART_PORT.println(motor2.encoderCount);
  //UART_PORT.println(motor3.encoderCount);
  //UART_PORT.println(digitalRead(motor3.encoderPinA));
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
void clearBlinkState(void) {
  digitalWrite(LED_BUILTIN, LOW);
  blinkCount = 0;
  sinceBlink = 0;
}
void blinkLED(void) {
  if (blinkType == HEARTBEAT && sinceBlink >= HEARTBEAT_PERIOD) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle pin state
    sinceBlink = 0;
  }
  else if (blinkType == GOOD_BLINK && sinceBlink >= GOOD_BLINK_PERIOD) {
    if (blinkCount >= MAX_GOOD_BLINKS) {
      blinkType = HEARTBEAT;
      clearBlinkState();
    }
    else {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle pin state
      sinceBlink = 0;
      if (digitalRead(LED_BUILTIN)) {
        blinkCount++;
      }
    }
  }
  else if (blinkType == BAD_BLINK && sinceBlink >= BAD_BLINK_PERIOD) {
    if (blinkCount >= MAX_BAD_BLINKS) {
      blinkType = HEARTBEAT;
      clearBlinkState();
    }
    else {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle pin state
      sinceBlink = 0;
      if (digitalRead(LED_BUILTIN)) {
        blinkCount++;
      }
    }
  }
  else { // do nothing
    ;
  }
}
void respondToLimitSwitches(void) {
  for (int i = 0; i < NUM_MOTORS; i++) { // I should maybe make a debouncer class?
    if (motorArray[i]->triggered) { // check if the switch was hit
      motorArray[i]->checkForActualPress();
    }
    if (motorArray[i]->actualPress) { // the switch was debounced and now we can react
#ifdef DEBUG_SWITCHES
      UART_PORT.print("ARM motor "); UART_PORT.print(i + 1);
      UART_PORT.println(" hit limit switch");
#endif
      if (i == 5) { // gripper switches
        motorArray[i]->stopRotation(); // stop turning of course
        if (motorArray[i]->limitSwitchState == CLOCKWISE) {
#ifdef DEBUG_SWITCHES
          UART_PORT.println("ARM gripper hit a switch");
          UART_PORT.print("ARM motor is at hard angle ");
          UART_PORT.print(motorArray[i]->maxSoftAngle);
#endif
          motorArray[i]->setSoftwareAngle(motorArray[i]->maxSoftAngle);
        }
        if (motorArray[i]->limitSwitchState == COUNTER_CLOCKWISE) {
#ifdef DEBUG_SWITCHES
          UART_PORT.println("ARM gripper hit a switch");
          UART_PORT.print("ARM motor is at hard angle ");
          UART_PORT.print(motorArray[i]->minSoftAngle);
#endif
          motorArray[i]->setSoftwareAngle(motorArray[i]->minSoftAngle);
        }
        motorArray[i]->actualPress = false;
        motorArray[i]->limitSwitchState = 0;
        if (isHoming) {
          motorArray[i]->homingDone = true; // just means that the switch was hit so homing round 2 can start if desired
        }
      }
      else {
        motorArray[i]->atSafeAngle = false;
        if (isHoming) {
          motorArray[i]->homingDone = true; // just means that the switch was hit so homing round 2 can start if desired
        }
        motorArray[i]->goToSafeAngle(); // internally stops movement and calls forceToAngle to overwrite previous command
      }
    }
    /*! \todo put code here to check if the motor should be at the end of its path but isn't?
       well how would it know if it isn't if it doesn't hit the limit switch because of software limits?
    */
  }
}
void homeArmMotors(void) { //!< \todo print homing debug just for motors which are homing
  if (homingMotor >= 0 && homingMotor < NUM_MOTORS) { // make sure it's a valid motor
    if (motorsToHome[homingMotor]) { // is this motor supposed to home?
      // the homing direction should be set-able based on the homing command if single direction (or even both i guess)
#ifdef DEBUG_HOMING
      UART_PORT.print("ARM homing motor "); UART_PORT.print(homingMotor + 1);
      UART_PORT.println(" inwards");
#endif
      motorArray[homingMotor]->homeMotor('i'); // start homing motor inwards
      motorsToHome[homingMotor] = false; // set this to false so it only happens once
    }
    if (motorArray[homingMotor]->homingDone) { // finished homing in a direction, set by motor timer interrupt
      if (motorArray[homingMotor]->atSafeAngle) { // makes sure that joint is in permissible range
        if (motorArray[homingMotor]->homingPass == 0) { // i can't see how this would ever be true?
#ifdef DEBUG_HOMING
          if (motorsToHome[homingMotor]) {
            UART_PORT.print("ARM motor "); UART_PORT.print(homingMotor + 1);
            UART_PORT.println(" homing 1 done and at safe angle");
          }
#endif
        }
        // will only home outwards if it's double ended homing, otherwise it moves on to the next motor
        if ( (motorArray[homingMotor]->homingType == DOUBLE_ENDED_HOMING) && (motorArray[homingMotor]->homingPass == 1) ) {
#ifdef DEBUG_HOMING
          UART_PORT.print("ARM homing motor "); UART_PORT.print(homingMotor + 1);
          UART_PORT.println(" outwards");
#endif
          motorArray[homingMotor]->homeMotor('o'); // start homing motor outwards
        }
        else { // done finding angle limits, moving to home position and then next motor time
          if (! (motorArray[homingMotor]->startedZeroing) ) { // start zeroing
#ifdef DEBUG_HOMING
            if (motorsToHome[homingMotor]) {
              UART_PORT.print("ARM motor "); UART_PORT.print(homingMotor + 1);
              UART_PORT.println(" homing complete. now to move to 0 degrees");
            }
#endif
            motorArray[homingMotor]->forceToAngle(0.0);
            motorArray[homingMotor]->startedZeroing = true;
          }
          else { // check to see if done zeroing yet, if so move on to next
            float angle = motorArray[homingMotor]->getSoftwareAngle();
            //float tolerance = motorArray[homingMotor]->pidController.getJointAngleTolerance();
            //if (fabs(angle) < tolerance * 3) { // within small enough angle range to move on to next motor
            if (fabs(angle) <= 1) { // changed angle for testing
#ifdef DEBUG_HOMING
              if (motorsToHome[homingMotor]) {
                UART_PORT.print("ARM motor "); UART_PORT.print(homingMotor + 1);
                UART_PORT.println(" zeroing complete.");
              }
#endif
              homingMotor--; // move on to the next motor
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
    UART_PORT.println("ARM all motors done homing, reinitializing motor timers");
#endif
    for (int i = 0; i < NUM_MOTORS; i++) {
      motorArray[i]->stopHoming();
    }
    isHoming = false;
    // interesting idea is to start homing the next motor while the previous one is finishing up
    // this would be a good place to call the goToNeutral function or whatever, or it should be its own thing for the sake of keeping things independent
    // motorArray[homingMotor]->forceToAngle(motorArray[homingMotor]->neutralAngle);
    homingMotor = -1; // reset it until next homing call
  }
}
void rebootTeensy(void) { //!< software reset function using watchdog timer
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  // The next 2 lines set the time-out value.
  WDOG_TOVALL = 15; // This is the value (ms) that the watchdog timer compare itself to.
  WDOG_TOVALH = 0; // End value (ms) WDT compares itself to.
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
                  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
  WDOG_PRESC = 0; //Sets watchdog timer to tick at 1 kHz inseast of 1/4 kHz
  while (1); // infinite do nothing loop -- wait for the countdown
}
/*! This function "kicks the dog". Refreshes its time-out counter. If not refreshed, system will be reset.
  This reset is triggered when the time-out value set in rebootTeensy() is exceeded.
  Calling the kickDog() function will reset the time-out counter.

  \todo Figure out where to implement kickDog() function into loop() and what time-out value to set for rebootTeensy().
*/
void kickDog(void) {
  /*
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
  */
}

/* timer interrupts */
void dcInterrupt(void) {
  motor1.motorTimerInterrupt();
  motor2.motorTimerInterrupt();
  motor3.motorTimerInterrupt();
  motor4.motorTimerInterrupt();
}
void servoInterrupt(void) {
  motor5.motorTimerInterrupt();
  motor6.motorTimerInterrupt();
}

/* encoder interrupts */
//#define M1_SINGLE_CHANNEL 1
#define M1_DUAL_CHANNEL 2
#ifdef M1_ENCODER_PORT
/*! encoder interrupt service routine

   \todo can I make a skeleton function that gets called in these interrupts
     that I can just pass the appropriate registers etc into?
*/
void m1_encoder_interrupt(void) {
#ifdef M1_DUAL_CHANNEL
  /*! encoder states are 4 bit values. Top 2 bits are the previous states
     of encoder channels A and B, bottom 2 are current states.
  */
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2; //!< shift current state into previous state
  /*! put the 2 relevant pin states from the relevant gpio port into memory, clearing the irrelevant bits
     this is done by 1) grabbing the input register of the port,
     2) shifting it until the relevant bits are in the lowest state,
     and 3) clearing all the bits higher than the lowest 2
     next, place said (current) pin states into the bottom 2 bits of oldEncoderState
  */
  oldEncoderState |= ((M1_ENCODER_PORT >> M1_ENCODER_SHIFT) & 0x03);
  /*! the encoderStates[] array corresponds to the correct direction
     for a specific set of prev and current encoder states.
     The & operation ensures that anything above the lowest 4 bits
     is cleared before accessing the array.
  */
  motor1.encoderCount += encoderStates[(oldEncoderState & 0x0F)] * motor1.encoderModifier;
#ifdef DEBUG_ENCODERS
  UART_PORT.print("ARM motor 1 "); UART_PORT.println(motor1.encoderCount);
#endif
#endif
#ifdef M1_SINGLE_CHANNEL
  //! use this version if only one encoder channel is working
  // this doesn't seem to work though so maybe don't... or fix it
  motor1.encoderCount += motor1.rotationDirection * 2;
#ifdef DEBUG_ENCODERS
  UART_PORT.print("ARM motor 1 "); UART_PORT.println(motor1.encoderCount);
#endif
#endif
}
#endif

#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)] * motor2.encoderModifier;
#ifdef DEBUG_ENCODERS
  UART_PORT.print("ARM motor 2 "); UART_PORT.println(motor2.encoderCount);
#endif
}
#endif
#ifdef M3_ENCODER_PORT
void m3_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M3_ENCODER_PORT >> M3_ENCODER_SHIFT) & 0x03);
  motor3.encoderCount += encoderStates[(oldEncoderState & 0x0F)] * motor3.encoderModifier;
#ifdef DEBUG_ENCODERS
  UART_PORT.print("ARM motor 3 "); UART_PORT.println(motor3.encoderCount);
#endif
}
#endif
#ifdef M4_ENCODER_PORT
void m4_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M4_ENCODER_PORT >> M4_ENCODER_SHIFT) & 0x03);
  motor4.encoderCount += encoderStates[(oldEncoderState & 0x0F)] * motor4.encoderModifier;
#ifdef DEBUG_ENCODERS
  UART_PORT.print("ARM motor 4 "); UART_PORT.println(motor4.encoderCount);
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
  UART_PORT.print("ARM motor 5 "); UART_PORT.println(motor5.encoderCount);
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
  UART_PORT.print("ARM motor 6 "); UART_PORT.println(motor6.encoderCount);
#endif
}
#endif

/* limit switch interrupts */
/* //! limit switch interrupt service routine
  \todo can I make a skeleton function that gets called in these interrupts
     that I can just pass the appropriate registers etc into?
*/
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
    UART_PORT.println("ARM m1 cw(right)");
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
    UART_PORT.println("ARM m1 ccw(left)");
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
    UART_PORT.println("ARM m2 cw(flex,down)");
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
    UART_PORT.println("ARM m2 ccw(ext,up)");
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
    UART_PORT.println("ARM m3 cw(flex,down)");
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
    UART_PORT.println("ARM m3 ccw(ext,up)");
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
    UART_PORT.println("ARM m4 cw(flex,down)");
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
    UART_PORT.println("ARM m4 ccw(ext,up)");
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
*/
void m6FlexISR(void) {
  // if the switch wasn't previously triggered then it starts a counter
  if (!(motor6.triggered) && motor6.triggerState == 0) {
    motor6.triggered = true;
    motor6.sinceTrigger = 0;
  }
  // grab the state of the pin for the cw switch
  int pinState = (M6_LIMIT_SW_FLEX_PORT >> M6_LIMIT_SW_FLEX_SHIFT ) & 1;
  // since we care about falling edges, when it reads 0 it's a hit
  // a hit is +1 or -1 depending on which switch was hit
  if (pinState == 0) {
    motor6.triggerState = CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("ARM m6 cw(open)");
#endif
  }
  // if it's not a hit we set it to 0
  else {
    motor6.triggerState = 0;
  }
}
void m6ExtendISR(void) {
  if (!(motor6.triggered) && motor6.triggerState == 0) {
    motor6.triggered = true;
    motor6.sinceTrigger = 0;
  }
  int pinState = (M6_LIMIT_SW_EXTEND_PORT >> M6_LIMIT_SW_EXTEND_SHIFT ) & 1;
  if (pinState == 0) {
    motor6.triggerState = COUNTER_CLOCKWISE;
#ifdef DEBUG_SWITCHES
    UART_PORT.println("ARM m6 ccw(close)");
#endif
  }
  else {
    motor6.triggerState = 0;
  }
}
