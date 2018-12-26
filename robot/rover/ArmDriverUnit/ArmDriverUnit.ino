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
This code began development sometime around July 20 2018 and is still being
updated as of December 22 2018.
*/
/* still in idea phase */
#define DEVEL1_MODE 1 // sends messages with Serial, everything unlocked
// #define DEVEL2_MODE 2 // sends messages with Serial1, everything unlocked
// #define DEBUG_MODE 3 // sends messages with ROSserial, everything unlocked
// #define USER_MODE 4 // sends messages with ROSserial, functionality restricted
// these switch on and off debugging but this should be modded to control serial1 vs 2 vs ROSserial
#define DEBUG_PARSING 10 // debug messages during parsing function
#define DEBUG_VERIFYING 11 // debug messages during verification function
#define DEBUG_LOOPING 12 // debug messages during main loop
// #define DEBUG_ENCODERS 13 // debug messages during encoder interrupts
#define DEBUG_SWITCHES 14 // debug messages during limit switch interrupts
#define DEBUG_TIMERS 15 // debug messages during timer interrupts
#define DEBUG_PID 16 // debug messages during pid loop calculations
/*
choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
should work.
*/
// serial communication over usb with pc, teensy not connected to odroid

#if defined(DEVEL1_MODE)
#define UART_PORT Serial
// serial communication over uart with odroid, teensy plugged into pcb and odroid
#elif defined(DEVEL2_MODE)
#define UART_PORT Serial1
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
#include "PinSetup.h"
#include "Parser.h"
// #include "Notes.h" // holds todo info
// #include "Ideas.h" // holds bits of code that haven't been implemented
#include "RobotMotor.h"
#include "StepperMotor.h"
#include "DcMotor.h"
#include "ServoMotor.h"
#define STEPPER_PID_PERIOD 25 * 1000 // initial value for constant speed, but adjusted in variable speed modes
#define DC_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define STEPPER_CHECK_INTERVAL 2000 // much longer period, used for testing/debugging
// #define STEPPER_CHECK_INTERVAL 250 // every 250ms check if the stepper is in the right spot
#define ENCODER_NVIC_PRIORITY 100 // should be higher priority than most other things, quick calculations and happens very frequently
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4 // lower priority than motors but still important
/* serial */
#define BAUD_RATE 115200 // serial baud rate
#define SERIAL_PRINT_INTERVAL 1000 // how often should teensy send angle data
#define SERIAL_READ_TIMEOUT 50 // how often should the serial port be read
#define BUFFER_SIZE 100 // size of the buffer for the serial commands
/* parsing */
char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
String messageBar = "=======================================================";
/*
info from parsing functionality is packaged and given to motor control functionality.
many of these are set to 0 so that the message can reset, thus making sure that
the code later on doesn't inadvertently make a motor move when it wasn't supposed to
*/
commandInfo motorCommand, emptyMotorCommand; // emptyMotorCommand is used to reset the struct when the loop restarts
Parser Parser;
// quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states
const int encoderStates[16] =
{
  0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
};

// instantiate motor objects here. only dcmotor currently supports interrupts
// StepperMotor motor1(M1_ENABLE_PIN, M1_DIR_PIN, M1_STEP_PIN, M1_STEP_RESOLUTION, FULL_STEP, M1_GEAR_RATIO);
DcMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO); // for cytron
// DcMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B); // sabertooth
DcMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO); // for cytron
StepperMotor motor3(M3_ENABLE_PIN, M3_DIR_PIN, M3_STEP_PIN, M3_STEP_RESOLUTION, FULL_STEP, M3_GEAR_RATIO);
StepperMotor motor4(M4_ENABLE_PIN, M4_DIR_PIN, M4_STEP_PIN, M4_STEP_RESOLUTION, FULL_STEP, M4_GEAR_RATIO);
ServoMotor motor5(M5_PWM_PIN, M5_GEAR_RATIO);
ServoMotor motor6(M6_PWM_PIN, M6_GEAR_RATIO);
// timer interrupts
IntervalTimer dcTimer; // motors 1&2
IntervalTimer m3StepperTimer;
IntervalTimer m4StepperTimer;
IntervalTimer servoTimer; // motors 5&6
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent
elapsedMillis sinceStepperCheck; // how long since last time stepper angle was verified
void printMotorAngles();
// declare encoder interrupt service routines. They must be global.

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

// declare timer interrupt service routines, where the motors actually get controlled.
// stepper interrupts occur much faster and the code is more complicated, so each stepper gets its own interrupt
void dcInterrupt(void); // manages motors 1&2
void m3StepperInterrupt(void);
void m4StepperInterrupt(void);
void servoInterrupt(void); // manages motors 5&6
void parseSerial(commandInfo & cmd); // goes through the message and puts relevant data into the motorCommand struct
bool verifSerial(commandInfo cmd); // error checks the parsed command
void setup()
{
  pinSetup(); // initializes all the appropriate pins to outputs or interrupt pins etc
  UART_PORT.begin(BAUD_RATE);
  UART_PORT.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms
  // each motor with an encoder needs to attach the encoder and 2 interrupts

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

  {
    // shaft angle tolerance setters
    // motor1.pidController.setJointAngleTolerance(1.8 * 3*motor1.gearRatioReciprocal); // if it was a stepper
    motor1.pidController.setJointAngleTolerance(2.0 * motor1.gearRatioReciprocal); // randomly chosen for dc
    motor2.pidController.setJointAngleTolerance(2.0 * motor2.gearRatioReciprocal);
    motor3.pidController.setJointAngleTolerance(1.8 * 2 * motor3.gearRatioReciprocal); // 1.8 is the min stepper resolution so I gave it +/- tolerance
    motor4.pidController.setJointAngleTolerance(1.8 * 2 * motor4.gearRatioReciprocal);
    motor5.pidController.setJointAngleTolerance(2.0 * motor5.gearRatioReciprocal); // randomly chosen for servo
    motor6.pidController.setJointAngleTolerance(2.0 * motor6.gearRatioReciprocal);
  }
  {
    // motor angle limit setters
    motor1.setAngleLimits(M1_MINIMUM_ANGLE, M1_MAXIMUM_ANGLE);
    motor2.setAngleLimits(M2_MINIMUM_ANGLE, M2_MAXIMUM_ANGLE);
    motor3.setAngleLimits(M3_MINIMUM_ANGLE, M3_MAXIMUM_ANGLE);
    motor4.setAngleLimits(M4_MINIMUM_ANGLE, M4_MAXIMUM_ANGLE);
    // motor5.setAngleLimits(M5_MINIMUM_ANGLE, M5_MAXIMUM_ANGLE); // this joint should be able to spin freely
    motor6.setAngleLimits(M6_MINIMUM_ANGLE, M6_MAXIMUM_ANGLE);
  }
  {
    // max (and min) closed loop speed setters, I limit it to 50% for safety.
    // Abtin thinks 50% should be a hard limit that can't be modified this easily
    motor1.pidController.setOutputLimits(-50, 50, 5.0);
    // motor2.pidController.setOutputLimits(-100, 100, 5.0);
    motor2.pidController.setOutputLimits(-50, 50, 5.0);
    motor3.pidController.setOutputLimits(-50, 50, 5.0);
    motor4.pidController.setOutputLimits(-50, 50, 5.0);
    motor5.pidController.setOutputLimits(-50, 50, 5.0);
    motor6.pidController.setOutputLimits(-50, 50, 5.0);
  }
  {
    // open loop setters. By default the motors are open loop, have constant velocity profiles (no ramping),
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
    motor4.isOpenLoop = true;
    motor4.hasRamping = false;
    motor4.openLoopSpeed = 50; // 50% speed
    motor5.isOpenLoop = true;
    motor5.hasRamping = false;
    motor5.openLoopSpeed = 50; // 50% speed
    motor5.openLoopGain = 1.0; // totally random guess, needs to be tested
    motor6.isOpenLoop = true;
    motor6.hasRamping = false;
    motor6.openLoopSpeed = 50; // 50% speed
    motor6.openLoopGain = 1.0; // totally random guess, needs to be tested
  }
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

void loop()
{
  motorCommand = emptyMotorCommand; // reset motorCommand so the microcontroller doesn't try to move a motor next loop
  if (UART_PORT.available())
  {
    // if a message was sent to the Teensy
    UART_PORT.println(messageBar);
    UART_PORT.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    UART_PORT.print("GOT: ");
    UART_PORT.println(serialBuffer); // send back what was received
    // Parser.parseCommand(motorCommand, bufferPointer);
    Parser.parseCommand(motorCommand, serialBuffer);
    // parseSerial(motorCommand); // goes through the message and puts the appropriate data into motorCommand struct
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    // restOfMessage = serialBuffer; // reset pointer
    if (!Parser.verifCommand(motorCommand))
    // if (!verifSerial(motorCommand))
    {
      UART_PORT.println("$E, verification failed");
    }
    else
    {
      UART_PORT.println("$S,Success: command verified");
      UART_PORT.println(messageBar);
      // emergency stop takes precedence
      if (motorCommand.stopAllMotors)
      {
        motor1.stopRotation();
        motor2.stopRotation();
        motor3.stopRotation();
        motor4.stopRotation();
        motor5.stopRotation();
        motor6.stopRotation();
        UART_PORT.println("all motors stopped because of emergency stop");
      }
      else
      {
        // stopping a single motor takes precedence
        if (motorCommand.stopSingleMotor)
        {
          switch (motorCommand.whichMotor)
          {
            case MOTOR1:
              motor1.stopRotation();
              break;
            case MOTOR2:
              motor2.stopRotation();
              break;
            case MOTOR3:
              motor3.stopRotation();
              break;
            case MOTOR4:
              motor4.stopRotation();
              break;
            case MOTOR5:
              motor5.stopRotation();
              break;
            case MOTOR6:
              motor6.stopRotation();
              break;
          }
          UART_PORT.print("stopped motor ");
          UART_PORT.println(motorCommand.whichMotor);
        }
        // make motors move
        else
          if (motorCommand.angleCommand)
          {
            UART_PORT.print("motor ");
            UART_PORT.println(motorCommand.whichMotor);
            UART_PORT.print(" desired angle (degrees) is: ");
            UART_PORT.println(motorCommand.whichAngle);
            UART_PORT.println(messageBar);
            switch (motorCommand.whichMotor)
            {
              // move a motor based on which one was commanded
              case MOTOR1:
              // this motor can swivel but has limits so it doesn't hit anything
              if (motor1.setDesiredAngle(motorCommand.whichAngle))
              {
                // this method returns true if the command is within joint angle limits
                if (motor1.isOpenLoop)
                {
                  motor1.openLoopError = motor1.getDesiredAngle(); // - motor1.calcCurrentAngle(); // find the angle difference
                  motor1.calcDirection(motor1.openLoopError); // determine rotation direction
                  // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
                  if (motor1.calcTurningDuration(motor1.openLoopError))
                  {
                    // returns false if the open loop error is too small
                    motor1.timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
                    motor1.movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
                    UART_PORT.print(motor1.numMillis);
                    UART_PORT.println(" milliseconds to turn");
                  }
                  else
                    UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor1.isOpenLoop)
                  {
                    // all the heavy lifting for closed loop control is done in the timer interrupt
                    motor1.movementDone = false;
                  }
              }
              else
                UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR2:// dc motor control using helper functions
              if (motor2.setDesiredAngle(motorCommand.whichAngle))
              {
                if (motor2.isOpenLoop)
                {
                  motor2.openLoopError = motor2.getDesiredAngle(); // - motor2.calcCurrentAngle();
                  motor2.calcDirection(motor2.openLoopError);
                  if (motor2.calcTurningDuration(motor2.openLoopError))
                  {
                    motor2.timeCount = 0;
                    motor2.movementDone = false;
                    UART_PORT.print(motor2.numMillis);
                    UART_PORT.println(" milliseconds to turn");
                  }
                  else
                    UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor2.isOpenLoop)
                  {
                    motor2.movementDone = false;
                  }
              }
              else
                UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR3:// stepper motor control using helper functions
              if (motor3.setDesiredAngle(motorCommand.whichAngle))
              {
                if (motor3.isOpenLoop)
                {
                  motor3.openLoopError = motor3.getDesiredAngle(); // - motor3.calcCurrentAngle(); // find the angle difference
                  motor3.calcDirection(motor3.openLoopError);
                  // calculates how many steps to take to get to the desired position, assuming no slipping
                  if (motor3.calcNumSteps(motor3.openLoopError))
                  {
                    // also returns false if the open loop error is too small
                    motor3.enablePower(); // give power to the stepper finally
                    motor3.movementDone = false;
                    UART_PORT.print(motor3.numSteps);
                    UART_PORT.println(" steps to turn");
                  }
                  else
                    UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor3.isOpenLoop)
                  {
                    motor3.enablePower();
                    motor3.movementDone = false;
                  }
              }
              else
                UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR4:
                if (motor4.setDesiredAngle(motorCommand.whichAngle))
                {
                  if (motor4.isOpenLoop)
                  {
                    motor4.openLoopError = motor4.getDesiredAngle(); // - motor4.calcCurrentAngle(); // find the angle difference
                    motor4.calcDirection(motor4.openLoopError);
                    if (motor4.calcNumSteps(motor4.openLoopError))
                    {
                      motor4.enablePower();
                      motor4.movementDone = false;
                      UART_PORT.print(motor4.numSteps);
                      UART_PORT.println(" steps to turn");
                    }
                    else
                      UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                  }
                  else
                    if (!motor4.isOpenLoop)
                    {
                      motor4.enablePower();
                      motor4.movementDone = false;
                    }
                }
                else
                  UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
                break;
              case MOTOR5:
              // this motor has no max or min angle because it must be able to spin like a screwdriver
              if (motor5.setDesiredAngle(motorCommand.whichAngle))
              {
                if (motor5.isOpenLoop)
                {
                  motor5.openLoopError = motor5.getDesiredAngle(); // - motor5.calcCurrentAngle(); // find the angle difference
                  motor5.calcDirection(motor5.openLoopError);
                  if (motor5.calcTurningDuration(motor5.openLoopError))
                  {
                    motor5.timeCount = 0;
                    motor5.movementDone = false;
                    UART_PORT.print(motor5.numMillis);
                    UART_PORT.println(" milliseconds to turn");
                  }
                  else
                    UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor5.isOpenLoop)
                  {
                    motor5.movementDone = false;
                  }
              }
              else
                UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR6:
                if (motor6.setDesiredAngle(motorCommand.whichAngle))
                {
                  if (motor6.isOpenLoop)
                  {
                    motor6.openLoopError = motor6.getDesiredAngle(); // - motor6.calcCurrentAngle(); // find the angle difference
                    motor6.calcDirection(motor6.openLoopError);
                    if (motor6.calcTurningDuration(motor6.openLoopError))
                    {
                      motor6.timeCount = 0;
                      motor6.movementDone = false;
                      UART_PORT.print(motor6.numMillis);
                      UART_PORT.println(" milliseconds to turn");
                    }
                    else
                      UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                  }
                  else
                    if (!motor6.isOpenLoop)
                    {
                      motor6.movementDone = false;
                    }
                }
                else
                  UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
                break;
            }
          }
        // set loop states for appropriate motor
        else
          if (motorCommand.loopCommand)
          {
            if (motorCommand.loopState == OPEN_LOOP)
            {
              switch (motorCommand.whichMotor)
              {
                case MOTOR1:
                  motor1.isOpenLoop = true;
                  break;
                case MOTOR2:
                  motor2.isOpenLoop = true;
                  break;
                case MOTOR3:
                  motor3.isOpenLoop = true;
                  break;
                case MOTOR4:
                  motor4.isOpenLoop = true;
                  break;
                case MOTOR5:
                  motor5.isOpenLoop = true;
                  break;
                case MOTOR6:
                  motor6.isOpenLoop = true;
                  break;
              }
              UART_PORT.print("motor ");
              UART_PORT.print(motorCommand.whichMotor);
              UART_PORT.println(" is open loop");
            }
            else
              if (motorCommand.loopState == CLOSED_LOOP)
              {
                switch (motorCommand.whichMotor)
                {
                  case MOTOR1:
                    if (motor1.hasEncoder)
                      motor1.isOpenLoop = false;
                    else
                      UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR2:
                    if (motor2.hasEncoder)
                      motor2.isOpenLoop = false;
                    else
                      UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR3:
                    if (motor3.hasEncoder)
                      motor3.isOpenLoop = false;
                    else
                      UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR4:
                    if (motor4.hasEncoder)
                      motor4.isOpenLoop = false;
                    else
                      UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR5:
                    if (motor5.hasEncoder)
                      motor5.isOpenLoop = false;
                    else
                      UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR6:
                    if (motor6.hasEncoder)
                      motor6.isOpenLoop = false;
                    else
                      UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                }
              }
          }
        // reset the motor angle's variable or actually control the motor to reset it to neutral position
        else
          if (motorCommand.resetCommand)
          {
            if (motorCommand.resetAngleValue)
            {
              switch (motorCommand.whichMotor)
              {
                case MOTOR1:
                  motor1.setCurrentAngle(0.0);
                  break;
                case MOTOR2:
                  motor2.setCurrentAngle(0.0);
                  break;
                case MOTOR3:
                  motor3.setCurrentAngle(0.0);
                  break;
                case MOTOR4:
                  motor4.setCurrentAngle(0.0);
                  break;
                case MOTOR5:
                  motor5.setCurrentAngle(0.0);
                  break;
                case MOTOR6:
                  motor6.setCurrentAngle(0.0);
                  break;
              }
              UART_PORT.print("reset angle value of motor ");
              UART_PORT.println(motorCommand.whichMotor);
            }
            else
              if (motorCommand.resetJointPosition)
              {
                ; // for later
              }
          }
        else
          UART_PORT.println("$E,Error: bad motor command");
      }
    }
  }
  if (sinceStepperCheck >= STEPPER_CHECK_INTERVAL)
  {
    /* this code could (should?) also disable power */
    // if (motor1.movementDone) motor1.disablePower();
    // if (motor3.movementDone) motor3.disablePower();
    // if (motor4.movementDone) motor4.disablePower();
    /* this code could (should?) determine when servo/dc motor movement is done */
    /*
    if ( fabs(motor2.desiredAngle - motor2.calcCurrentAngle() ) < motor2.pidController.jointAngleTolerance){
    motor2.movementDone = true;
    }
    */
    /*
    if ( fabs(motor5.desiredAngle - motor5.calcCurrentAngle() ) < motor5.pidController.jointAngleTolerance){
    motor5.movementDone = true;
    }
    if ( fabs(motor6.desiredAngle - motor6.calcCurrentAngle() ) < motor6.pidController.jointAngleTolerance){
    motor6.movementDone = true;
    }
    */
    // all of this code should probably go into a function called "calculate motor steps" or something...
    // this code is very similar to what happens above when it decides which motor should turn after receiving a command
    // this code also assumes that the correct amount of steps will take it to the right spot
    // this means that it doesn't account for faulty angle calculations from the encoder or the motor resolution...
    /*
    //motor4
    int m4remainingSteps = motor4.numSteps - motor4.stepCount;
    //float m4imaginedAngle = motor4.stepCount * motor4.stepResolution * motor4.gearRatioReciprocal;
    //float m4actualAngle = motor4.calcCurrentAngle();
    float m4imaginedRemainingAngle = m4remainingSteps * motor4.stepResolution * motor4.gearRatioReciprocal; // how far does the motor think it needs to go
    float m4actualRemainingAngle = motor4.desiredAngle - motor4.calcCurrentAngle(); // how far does it actually need to go
    float m4discrepancy = m4actualRemainingAngle - m4imaginedRemainingAngle ;
    //motor3
    int remainingSteps = motor3.numSteps - motor3.stepCount;
    //float imaginedAngle = motor3.stepCount * motor3.stepResolution * motor3.gearRatioReciprocal;
    //float actualAngle = motor3.calcCurrentAngle();
    float imaginedRemainingAngle = remainingSteps * motor3.stepResolution * motor3.gearRatioReciprocal; // how far does the motor think it needs to go
    float actualRemainingAngle = motor3.desiredAngle - motor3.calcCurrentAngle(); // how far does it actually need to go
    float discrepancy = actualRemainingAngle - imaginedRemainingAngle ;
    */
    // UART_PORT.print(imaginedAngle); UART_PORT.println(" imagined angle");
    // UART_PORT.print(actualAngle); UART_PORT.println(" actual angle");
    // UART_PORT.print(imaginedRemainingAngle); UART_PORT.println(" imagined remaining angle");
    // UART_PORT.print(actualRemainingAngle); UART_PORT.println(" actual remaining angle");
    // UART_PORT.print(discrepancy); UART_PORT.println(" degrees behind expected position");
    /*
    // the stepper interrupt could occur during this calculation, so maybe there should be a different angle tolerance here
    // that said at the moment it's 2 degrees which is bigger than the max step angle of the motor
    // keep in mind that 2 degrees for the joint is different from 2 degrees for the motor shaft
    if (fabs(discrepancy) > motor3.pidController.jointAngleTolerance) {
    UART_PORT.println("discrepancy is too high and motor is moving, adjusting step number");
    // it's possible the check happens during movement, but there needs to be ample distance to move
    if (!motor3.movementDone) {
    // if actualRemainingAngle is negative it means the arm moved way further than it should have
    if(actualRemainingAngle<0) motor3.movementDone=true; // abort
    else if(actualRemainingAngle > motor3.pidController.jointAngleTolerance){
    UART_PORT.println("enough angle between current position and desired position to adjust during movement");
    // the adjustment is simple if the motor is already moving in the right direction but what happens when a direction change needs to occur?
    // the motor interrupt assumes the step count and direction are unchanged!!!!
    motor3.numSteps += discrepancy * motor3.gearRatio / motor3.stepResolution; // add the number of steps required to catch up or skip
    //numsteps gets updated but imagined angle doesnt...?
    }
    else UART_PORT.println("not enough angle between current position and desired position to adjust during movement, waiting for movement to end");
    }
    else { // it's possible the check happens when the motor is at rest
    UART_PORT.println("discrepancy is too high and motor is done moving, adjusting step number");
    // it's possible the angle is too far in either direction, so this makes sure that it goes to the right spot
    if (discrepancy >= 0) motor3.rotationDirection = 1;
    else discrepancy = -1;
    motor3.enablePower();
    motor3.numSteps = fabs(discrepancy) * motor3.gearRatio / motor3.stepResolution; // calculate the number of steps to take
    motor3.movementDone = false;
    }
    }
    */
    sinceStepperCheck = 0;
  }
  // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL)
  {
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }
}

void printMotorAngles()
{
  UART_PORT.print("Motor Angles: ");
  if (motor1.calcCurrentAngle())
  {
    UART_PORT.print(motor1.getCurrentAngle());
  }
  if (motor2.calcCurrentAngle())
  {
    UART_PORT.print(",");
    UART_PORT.print(motor2.getCurrentAngle());
  }
  if (motor3.calcCurrentAngle())
  {
    UART_PORT.print(",");
    UART_PORT.print(motor3.getCurrentAngle());
  }
  if (motor4.calcCurrentAngle())
  {
    UART_PORT.print(",");
    UART_PORT.print(motor4.getCurrentAngle());
  }
  if (motor5.calcCurrentAngle())
  {
    UART_PORT.print(",");
    UART_PORT.print(motor5.getCurrentAngle());
  }
  if (motor6.calcCurrentAngle())
  {
    UART_PORT.print(",");
    UART_PORT.print(motor6.getCurrentAngle());
  }
  UART_PORT.println("");
}

void m3StepperInterrupt(void)
{
  motor3.nextInterval = STEPPER_PID_PERIOD; // how long until the next step is taken? indirectly controls speed
  if (motor3.isOpenLoop)
  {
    // open loop control
    // movementDone can be set elsewhere... so can numSteps
    if (!motor3.movementDone && motor3.stepCount < motor3.numSteps)
    {
      motor3.setVelocity(motor3.rotationDirection, motor3.openLoopSpeed); // direction was set beforehand
      motor3.stepCount++;
      if (motor3.hasRamping)
      {
        // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        // nextInterval = STEPPER_PID_PERIOD; // replace with accessing a motor-specific array
      }
      m3StepperTimer.update(motor3.nextInterval); // need to check if can call this inside the interrupt
      // UART_PORT.print(motor3.rotationDirection); UART_PORT.println(" direction");
      // UART_PORT.print(motor3.stepCount); UART_PORT.println(" steps taken");
      // UART_PORT.print(motor3.numSteps); UART_PORT.println(" steps total");
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor3.stepCount = 0; // reset the counter
      motor3.movementDone = true;
      motor3.stopRotation();
      motor3.nextInterval = STEPPER_PID_PERIOD; // set it back to default
    }
  }
  else
    if (!motor3.isOpenLoop)
    {
      // closed loop control
      if (!motor3.movementDone)
      {
        motor3.calcCurrentAngle();
        motor3.pidController.updatePID(motor3.getCurrentAngle(), motor3.getDesiredAngle());
        if (motor3.pidController.pidOutput == 0)
        {
          motor3.movementDone = true;
          motor3.stopRotation();
          motor3.nextInterval = STEPPER_PID_PERIOD; // set it back to default
        }
        else
        {
          motor3.calcDirection(motor3.pidController.pidOutput); // does this work? it expects an angular error but at the end of the day...
          motor3.setVelocity(motor3.rotationDirection, motor3.pidController.pidOutput);
          m3StepperTimer.update(motor3.nextInterval); // need to check if can call this inside the interrupt
        }
      }
      else
      {
        motor3.stopRotation();
      }
    }
}

void m4StepperInterrupt(void)
{
  motor4.nextInterval = STEPPER_PID_PERIOD;
  if (motor4.isOpenLoop)
  {
    // open loop control
    if (!motor4.movementDone && motor4.stepCount < motor4.numSteps)
    {
      motor4.setVelocity(motor4.rotationDirection, motor4.openLoopSpeed); // direction was set beforehand
      motor4.stepCount++;
      if (motor4.hasRamping)
      {
        // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        // nextInterval = STEPPER_PID_PERIOD; // replace with accessing a motor-specific array
      }
      m4StepperTimer.update(motor4.nextInterval); // need to check if can call this inside the interrupt
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor4.stepCount = 0; // reset the counter
      motor4.movementDone = true;
      motor4.stopRotation();
      motor4.nextInterval = STEPPER_PID_PERIOD; // set it back to default
    }
  }
  else
    if (!motor4.isOpenLoop)
    {
      // closed loop control
      if (!motor4.movementDone)
      {
        motor4.calcCurrentAngle();
        motor4.pidController.updatePID(motor4.getCurrentAngle(), motor4.getDesiredAngle());
        if (motor4.pidController.pidOutput == 0)
        {
          motor4.movementDone = true;
          motor4.stopRotation();
          motor4.nextInterval = STEPPER_PID_PERIOD; // set it back to default
        }
        else
        {
          motor4.calcDirection(motor4.pidController.pidOutput); // does this work? it expects an angular error but at the end of the day...
          motor4.setVelocity(motor4.rotationDirection, motor4.pidController.pidOutput);
          m4StepperTimer.update(motor4.nextInterval); // need to check if can call this inside the interrupt
        }
      }
      else
      {
        motor4.stopRotation();
      }
    }
}

void dcInterrupt(void)
{
  // movementDone can be set elsewhere... so are numMillis,
  // openLoopSpeed and rotationDirection (in open loop control)
  // motor 1
  if (motor1.isOpenLoop)
  {
    // open loop control
    if (!motor1.movementDone && motor1.timeCount <= motor1.numMillis)
    {
      motor1.setVelocity(motor1.rotationDirection, motor1.openLoopSpeed);
    }
    else
    {
      motor1.movementDone = true;
      motor1.stopRotation();
    }
  }
  // would be nice to have some kind of check for the above functions so the command only runs if there's been a change
  // e.g. movementDone changed or the speed or numMillis changed
  else
    if (!motor1.isOpenLoop)
    {
      if (!motor1.movementDone)
      {
        motor1.calcCurrentAngle();
        motor1.pidController.updatePID(motor1.getCurrentAngle(), motor1.getDesiredAngle());
        if (motor1.pidController.pidOutput == 0)
        {
          motor1.movementDone = true;
          motor1.stopRotation();
        }
        else
        {
          motor1.calcDirection(motor1.pidController.pidOutput); // does this work? it expects an angular error but at the end of the day...
          motor1.setVelocity(motor1.rotationDirection, motor1.pidController.pidOutput);
        }
      }
      else
      {
        motor1.stopRotation();
      }
    }
  // motor 2
  if (motor2.isOpenLoop)
  {
    // open loop control
    if (!motor2.movementDone && motor2.timeCount <= motor2.numMillis)
    {
      motor2.setVelocity(motor2.rotationDirection, motor2.openLoopSpeed);
    }
    else
    {
      motor2.movementDone = true;
      motor2.stopRotation();
    }
  }
  // would be nice to have some kind of check for the above functions so the command only runs if there's been a change
  // e.g. movementDone changed or the speed or numMillis changed
  else
    if (!motor2.isOpenLoop)
    {
      if (!motor2.movementDone)
      {
        motor2.calcCurrentAngle();
        motor2.pidController.updatePID(motor2.getCurrentAngle(), motor2.getDesiredAngle());
        if (motor2.pidController.pidOutput == 0)
        {
          motor2.movementDone = true;
          motor2.stopRotation();
        }
        else
        {
          motor2.calcDirection(motor2.pidController.pidOutput); // does this work? it expects an angular error but at the end of the day...
          motor2.setVelocity(motor2.rotationDirection, motor2.pidController.pidOutput);
        }
      }
      else
      {
        motor2.stopRotation();
      }
    }
}

void servoInterrupt(void)
{
  // movementDone can be set elsewhere... so are numMillis,
  // openLoopSpeed and rotationDirection (in open loop control)numMillis
  // motor 5
  if (motor5.isOpenLoop)
  {
    // open loop control
    if (!motor5.movementDone && motor5.timeCount < motor5.numMillis)
    {
      motor5.setVelocity(motor5.rotationDirection, motor5.openLoopSpeed);
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor5.movementDone = true;
      motor5.stopRotation();
    }
  }
  // the below will only work if the servo has encoders
  else
    if (!motor5.isOpenLoop)
    {
      if (!motor5.movementDone)
      {
        motor5.calcCurrentAngle();
        motor5.pidController.updatePID(motor5.getCurrentAngle(), motor5.getDesiredAngle());
        if (motor5.pidController.pidOutput == 0)
        {
          motor5.movementDone = true;
          motor5.stopRotation();
        }
        else
        {
          motor5.calcDirection(motor5.pidController.pidOutput); // does this work? it expects an angular error but at the end of the day...
          motor5.setVelocity(motor5.rotationDirection, motor5.pidController.pidOutput);
        }
      }
      else
      {
        motor5.stopRotation();
      }
    }
  // motor 6
  if (motor6.isOpenLoop)
  {
    // open loop control
    if (!motor6.movementDone && motor6.timeCount < motor6.numMillis)
    {
      motor6.setVelocity(motor6.rotationDirection, motor6.openLoopSpeed);
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor6.movementDone = true;
      motor6.stopRotation();
    }
  }
  // the below will only work if the servo has encoders
  else
    if (!motor6.isOpenLoop)
    {
      if (!motor6.movementDone)
      {
        motor6.calcCurrentAngle();
        motor6.pidController.updatePID(motor6.getCurrentAngle(), motor6.getDesiredAngle());
        if (motor6.pidController.pidOutput == 0)
        {
          motor6.movementDone = true;
          motor6.stopRotation();
        }
        else
        {
          motor6.calcDirection(motor6.pidController.pidOutput); // does this work? it expects an angular error but at the end of the day...
          motor6.setVelocity(motor6.rotationDirection, motor6.pidController.pidOutput);
        }
      }
      else
      {
        motor6.stopRotation();
      }
    }
}

#ifdef M1_ENCODER_PORT
void m1_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;

  #ifdef DEBUG_ENCODERS
  UART_PORT.print("m1 ");
  UART_PORT.println(motor1.encoderCount);
  #endif

  oldEncoderState <<= 2; // move by two bits (previous state in top 2 bits)
  oldEncoderState |= ((M1_ENCODER_PORT >> M1_ENCODER_SHIFT) & 0x03);
  /*
  encoderPort corresponds to the state of all the pins on the port this encoder is connected to.
  shift it right by the amount previously determined based on the encoder pin and the corresponding internal GPIO bit
  now the current state is in the lowest 2 bits, so you clear the higher bits by doing a logical AND with 0x03 (0b00000011)
  you then logical OR this with the previous state's shifted form to obtain (prevstate 1 prevstate 2 currstate 1 currstate 2)
  the catch which is accounted for below is that oldEncoderState keeps getting right-shifted so you need to clear the higher bits after this operation too
  */
  // clear the higher bits. The dir[] array corresponds to the correct direction for a specific set of prev and current encoder states
  motor1.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

#endif

#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;

  #ifdef DEBUG_ENCODERS
  UART_PORT.print("m2 ");
  UART_PORT.println(motor2.encoderCount);
  #endif

  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

#endif

#ifdef M3_ENCODER_PORT
void m3_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;

  #ifdef DEBUG_ENCODERS
  UART_PORT.println("m3 ");
  UART_PORT.println(motor3.encoderCount);
  // UART_PORT.println(M3_ENCODER_PORT,BIN);
  #endif

  oldEncoderState <<= 2;
  oldEncoderState |= ((M3_ENCODER_PORT >> M3_ENCODER_SHIFT) & 0x03);
  motor3.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

#endif

#ifdef M4_ENCODER_PORT
void m4_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;

  #ifdef DEBUG_ENCODERS
  UART_PORT.print("m4 ");
  UART_PORT.println(motor4.encoderCount);
  #endif

  oldEncoderState <<= 2;
  oldEncoderState |= ((M4_ENCODER_PORT >> M4_ENCODER_SHIFT) & 0x03);
  motor4.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

#endif

#ifdef M5_ENCODER_PORT
void m5_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;

  #ifdef DEBUG_ENCODERS
  UART_PORT.print("m5 ");
  UART_PORT.println(motor5.encoderCount);
  #endif

  oldEncoderState <<= 2;
  oldEncoderState |= ((M5_ENCODER_PORT >> M5_ENCODER_SHIFT) & 0x03);
  motor5.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

#endif

#ifdef M6_ENCODER_PORT
void m6_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;

  #ifdef DEBUG_ENCODERS
  UART_PORT.print("m6 ");
  UART_PORT.println(motor6.encoderCount);
  #endif

  oldEncoderState <<= 2;
  oldEncoderState |= ((M6_ENCODER_PORT >> M6_ENCODER_SHIFT) & 0x03);
  motor6.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

#endif
