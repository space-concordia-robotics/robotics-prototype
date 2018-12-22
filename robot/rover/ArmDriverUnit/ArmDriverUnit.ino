/*
Description of code goes here.
This code began development sometime around July 20 2018 and is still being updated as of December 17 2018.
*/
#include "PinSetup.h"
#include "Notes.h" // holds todo info
#include "Ideas.h" // holds bits of code that haven't been implemented
#include "RobotMotor.h"
#include "StepperMotor.h"
#include "DcMotor.h"
#include "ServoMotor.h"
/* still in idea phase */
#define DEVEL1_MODE 1 // sends messages with Serial, everything unlocked
#define DEVEL2_MODE 2 // sends messages with Serial1, everything unlocked
#define DEBUG_MODE 3 // sends messages with ROSserial, everything unlocked
#define USER_MODE 4 // sends messages with ROSserial, functionality restricted
#define DEBUG_PARSING 10 // debug messages during parsing function
#define DEBUG_VERIFYING 11 // debug messages during verification function
#define DEBUG_LOOPING 12 // debug messages during main loop
#define DEBUG_ENCODERS 13 // debug messages during encoder interrupts
#define DEBUG_SWITCHES 14 // debug messages during limit switch interrupts
#define DEBUG_TIMERS 15 // debug messages during timer interrupts
/* there are 2 different potentially conflicting ideas here:
1) compiling only creates machine code for the type of serial communication that's actually necessary.
this way extra serial.print statements won't be compiled when it goes on the teensy unless necessary.
that said, it might be good to leave this option open to make sure that it works even without ROS.
2) in regular operation, the user shouldn't be able to adjust pid constants or gear ratios and such.
but they should be allowed to unlock these features if they need to. this only applies to the final stage
when the rover is out on the field. At this stage it shouldn't be necessary to use regular serial.print?
but maybe it should be available just in case...
*/
#define STEPPER_PID_PERIOD 30 * 1000 // initial value for constant speed, but adjusted in variable speed modes
#define DC_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 20000 // 20ms, because typical pwm signals have 20ms periods
#define STEPPER_CHECK_INTERVAL 2000 // much longer period, used for testing/debugging
// #define STEPPER_CHECK_INTERVAL 250 // every 250ms check if the stepper is in the right spot
#define ENCODER_NVIC_PRIORITY 100 // should be higher priority tan most other things, quick calculations and happens very frequently
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4 // lower priority than motors but still important
/* serial */
#define BAUD_RATE 115200 // serial baud rate
#define SERIAL_PRINT_INTERVAL 1000 // how often should teensy send angle data
#define SERIAL_READ_TIMEOUT 50 // how often should the serial port be read
#define BUFFER_SIZE 100 // size of the buffer for the serial commands
/* parsing */
char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
char * restOfMessage = serialBuffer; // used in strtok_r, which is the reentrant version of strtok
String messageBar = "=======================================================";
/*
info from parsing functionality is packaged and given to motor control functionality.
many of these are set to 0 so that the message can reset, thus making sure that
the code later on doesn't inadvertently make a motor move when it wasn't supposed to
*/
struct commandInfo
{
  int whichMotor = 0; // which motor was requested to do something
  int whichDirection = 0; // set the direction
  int whichSpeed = 0; // set the speed
  unsigned int whichTime = 0; // how long to turn for
  bool angleCommand = false; // for regular operations, indicates that it's to control an angle
  float whichAngle = 0.0; // for regular operations, which angle to go to
  bool loopCommand = false; // for choosing between open loop or closed loop control
  int loopState = 0; // what type of loop state it is
  bool resetCommand = false; // indicates that something should be reset
  bool resetAngleValue = false; // mostly for debugging/testing, reset the angle variable
  bool resetJointPosition = false; // for moving a joint to its neutral position
  bool stopSingleMotor = false; // for stopping a single motor
  bool stopAllMotors = false; // for stopping all motors
}

motorCommand, emptyMotorCommand; // emptyMotorCommand is used to reset the struct when the loop restarts
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
void m1_encoder_interrupt(void);
void m2_encoder_interrupt(void);
void m3_encoder_interrupt(void);
void m4_encoder_interrupt(void);
// void m5_encoder_interrupt(void);
// void m6_encoder_interrupt(void);
// declare timer interrupt service routines, where the motors actually get controlled.
// stepper interrupts occur much faster and the code is more complicated, so each stepper gets its own interrupt
void dcInterrupt(void); // manages motors 1&2
void m3StepperInterrupt(void);
void m4StepperInterrupt(void);
void servoInterrupt(void); // manages motors 5&6
void parseSerial(commandInfo& cmd); // goes through the message and puts relevant data into the motorCommand struct
bool verifSerial(commandInfo cmd); // error checks the parsed command
void setup()
{
  pinSetup(); // initializes all the appropriate pins to outputs or interrupt pins etc
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms
  // each motor with an encoder needs to setup that encoder
  motor1.attachEncoder(M1_ENCODER_A, M1_ENCODER_B, M1_ENCODER_PORT, M1_ENCODER_SHIFT, M1_ENCODER_RESOLUTION);
  motor2.attachEncoder(M2_ENCODER_A, M2_ENCODER_B, M2_ENCODER_PORT, M2_ENCODER_SHIFT, M2_ENCODER_RESOLUTION);
  motor3.attachEncoder(M3_ENCODER_A, M3_ENCODER_B, M3_ENCODER_PORT, M3_ENCODER_SHIFT, M3_ENCODER_RESOLUTION);
  motor4.attachEncoder(M4_ENCODER_A, M4_ENCODER_B, M4_ENCODER_PORT, M4_ENCODER_SHIFT, M4_ENCODER_RESOLUTION);
  // each motor needs to attach 2 interrupts, which is a lot of lines of code
  attachInterrupt(motor1.encoderPinA, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor1.encoderPinB, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinA, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinB, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinA, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinB, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinA, m4_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinB, m4_encoder_interrupt, CHANGE);
  // attachInterrupt(motor5.encoderPinA, m5_encoder_interrupt, CHANGE);
  // attachInterrupt(motor5.encoderPinB, m5_encoder_interrupt, CHANGE);
  // attachInterrupt(motor6.encoderPinA, m6_encoder_interrupt, CHANGE);
  // attachInterrupt(motor6.encoderPinB, m6_encoder_interrupt, CHANGE);
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
    // max (and min) speed setters, I limit it to 50% for safety.
    // Abtin thinks 50% should be a hard limit that can't be modified this easily
    motor1.pidController.setOutputLimits(-50, 50);
    motor2.pidController.setOutputLimits(-50, 50);
    motor3.pidController.setOutputLimits(-50, 50);
    motor4.pidController.setOutputLimits(-50, 50);
    motor5.pidController.setOutputLimits(-50, 50);
    motor6.pidController.setOutputLimits(-50, 50);
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
  if (Serial.available())
  {
    // if a message was sent to the Teensy
    Serial.println(messageBar);
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: ");
    Serial.println(serialBuffer); // send back what was received
    parseSerial(motorCommand); // goes through the message and puts the appropriate data into motorCommand struct
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    restOfMessage = serialBuffer; // reset pointer
    if (!verifSerial(motorCommand))
    {
      Serial.println("$E, verification failed");
    }
    else
    {
      Serial.println("$S, command verified");
      Serial.println(messageBar);
      // emergency stop takes precedence
      if (motorCommand.stopAllMotors)
      {
        motor1.stopRotation();
        motor2.stopRotation();
        motor3.stopRotation();
        motor4.stopRotation();
        motor5.stopRotation();
        motor6.stopRotation();
        Serial.println("all motors stopped because of emergency stop");
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
          Serial.print("stopped motor ");
          Serial.println(motorCommand.whichMotor);
        }
        // make motors move
        else
          if (motorCommand.angleCommand)
          {
            Serial.print("motor ");
            Serial.println(motorCommand.whichMotor);
            Serial.print(" desired angle (degrees) is: ");
            Serial.println(motorCommand.whichAngle);
            Serial.println(messageBar);
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
                  motor1.openLoopError = motor1.desiredAngle; // - motor1.calcCurrentAngle(); // find the angle difference
                  motor1.calcDirection(motor1.openLoopError); // determine rotation direction
                  // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
                  if (motor1.calcTurningDuration(motor1.openLoopError))
                  {
                    // returns false if the open loop error is too small
                    motor1.timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
                    motor1.movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
                    Serial.print(motor1.numMillis);
                    Serial.println(" milliseconds to turn");
                  }
                  else
                    Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor1.isOpenLoop)
                  {
                    // all the heavy lifting for closed loop control is done in the timer interrupt
                    motor1.movementDone = false;
                  }
              }
              else
                Serial.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR2:// dc motor control using helper functions
              if (motor2.setDesiredAngle(motorCommand.whichAngle))
              {
                if (motor2.isOpenLoop)
                {
                  motor2.openLoopError = motor2.desiredAngle; // - motor2.calcCurrentAngle();
                  motor2.calcDirection(motor2.openLoopError);
                  if (motor2.calcTurningDuration(motor2.openLoopError))
                  {
                    motor2.timeCount = 0;
                    motor2.movementDone = false;
                    Serial.print(motor2.numMillis);
                    Serial.println(" milliseconds to turn");
                  }
                  else
                    Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor2.isOpenLoop)
                  {
                    motor2.movementDone = false;
                  }
              }
              else
                Serial.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR3:// stepper motor control using helper functions
              if (motor3.setDesiredAngle(motorCommand.whichAngle))
              {
                if (motor3.isOpenLoop)
                {
                  motor3.openLoopError = motor3.desiredAngle; // - motor3.calcCurrentAngle(); // find the angle difference
                  motor3.calcDirection(motor3.openLoopError);
                  // calculates how many steps to take to get to the desired position, assuming no slipping
                  if (motor3.calcNumSteps(motor3.openLoopError))
                  {
                    // also returns false if the open loop error is too small
                    motor3.enablePower(); // give power to the stepper finally
                    motor3.movementDone = false;
                    Serial.print(motor3.numSteps);
                    Serial.println(" steps to turn");
                  }
                  else
                    Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor3.isOpenLoop)
                  {
                    ; // commands for open loop if necessary
                    // motor4.movementDone = false;
                  }
              }
              else
                Serial.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR4:
                if (motor4.setDesiredAngle(motorCommand.whichAngle))
                {
                  if (motor4.isOpenLoop)
                  {
                    motor4.openLoopError = motor4.desiredAngle; // - motor4.calcCurrentAngle(); // find the angle difference
                    motor4.calcDirection(motor4.openLoopError);
                    if (motor4.calcNumSteps(motor4.openLoopError))
                    {
                      motor4.enablePower();
                      motor4.movementDone = false;
                      Serial.print(motor4.numSteps);
                      Serial.println(" steps to turn");
                    }
                    else
                      Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                  }
                  else
                    if (!motor4.isOpenLoop)
                    {
                      ; // commands for open loop if necessary
                      // motor4.movementDone = false;
                    }
                }
                else
                  Serial.println("$E,Alert: requested angle is not within angle limits.");
                break;
              case MOTOR5:
              // this motor has no max or min angle because it must be able to spin like a screwdriver
              if (motor5.setDesiredAngle(motorCommand.whichAngle))
              {
                if (motor5.isOpenLoop)
                {
                  motor5.openLoopError = motor5.desiredAngle; // - motor5.calcCurrentAngle(); // find the angle difference
                  motor5.calcDirection(motor5.openLoopError);
                  if (motor5.calcTurningDuration(motor5.openLoopError))
                  {
                    motor5.timeCount = 0;
                    motor5.movementDone = false;
                    Serial.print(motor5.numMillis);
                    Serial.println(" milliseconds to turn");
                  }
                  else
                    Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                }
                else
                  if (!motor5.isOpenLoop)
                  {
                    motor5.movementDone = false;
                  }
              }
              else
                Serial.println("$E,Alert: requested angle is not within angle limits.");
              break;
              case MOTOR6:
                if (motor6.setDesiredAngle(motorCommand.whichAngle))
                {
                  if (motor6.isOpenLoop)
                  {
                    motor6.openLoopError = motor6.desiredAngle; // - motor6.calcCurrentAngle(); // find the angle difference
                    motor6.calcDirection(motor6.openLoopError);
                    if (motor6.calcTurningDuration(motor6.openLoopError))
                    {
                      motor6.timeCount = 0;
                      motor6.movementDone = false;
                      Serial.print(motor6.numMillis);
                      Serial.println(" milliseconds to turn");
                    }
                    else
                      Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                  }
                  else
                    if (!motor6.isOpenLoop)
                    {
                      motor6.movementDone = false;
                    }
                }
                else
                  Serial.println("$E,Alert: requested angle is not within angle limits.");
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
              Serial.print("motor ");
              Serial.print(motorCommand.whichMotor);
              Serial.println(" is open loop");
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
                      Serial.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR2:
                    if (motor2.hasEncoder)
                      motor2.isOpenLoop = false;
                    else
                      Serial.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR3:
                    if (motor3.hasEncoder)
                      motor3.isOpenLoop = false;
                    else
                      Serial.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR4:
                    if (motor4.hasEncoder)
                      motor4.isOpenLoop = false;
                    else
                      Serial.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR5:
                    if (motor5.hasEncoder)
                      motor5.isOpenLoop = false;
                    else
                      Serial.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                    break;
                  case MOTOR6:
                    if (motor6.hasEncoder)
                      motor6.isOpenLoop = false;
                    else
                      Serial.println("$E,Alert: cannot use closed loop if motor has no encoder.");
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
                  motor1.currentAngle = 0.0;
                  break;
                case MOTOR2:
                  motor2.currentAngle = 0.0;
                  break;
                case MOTOR3:
                  motor3.currentAngle = 0.0;
                  break;
                case MOTOR4:
                  motor4.currentAngle = 0.0;
                  break;
                case MOTOR5:
                  motor5.currentAngle = 0.0;
                  break;
                case MOTOR6:
                  motor6.currentAngle = 0.0;
                  break;
              }
              Serial.print("reset angle value of motor ");
              Serial.println(motorCommand.whichMotor);
            }
            else
              if (motorCommand.resetJointPosition)
              {
                ; // for later
              }
          }
        else
          Serial.println("$E,Error: bad motor command");
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
    // Serial.print(imaginedAngle); Serial.println(" imagined angle");
    // Serial.print(actualAngle); Serial.println(" actual angle");
    // Serial.print(imaginedRemainingAngle); Serial.println(" imagined remaining angle");
    // Serial.print(actualRemainingAngle); Serial.println(" actual remaining angle");
    // Serial.print(discrepancy); Serial.println(" degrees behind expected position");
    /*
    // the stepper interrupt could occur during this calculation, so maybe there should be a different angle tolerance here
    // that said at the moment it's 2 degrees which is bigger than the max step angle of the motor
    // keep in mind that 2 degrees for the joint is different from 2 degrees for the motor shaft
    if (fabs(discrepancy) > motor3.pidController.jointAngleTolerance) {
    Serial.println("discrepancy is too high and motor is moving, adjusting step number");
    // it's possible the check happens during movement, but there needs to be ample distance to move
    if (!motor3.movementDone) {
    // if actualRemainingAngle is negative it means the arm moved way further than it should have
    if(actualRemainingAngle<0) motor3.movementDone=true; // abort
    else if(actualRemainingAngle > motor3.pidController.jointAngleTolerance){
    Serial.println("enough angle between current position and desired position to adjust during movement");
    // the adjustment is simple if the motor is already moving in the right direction but what happens when a direction change needs to occur?
    // the motor interrupt assumes the step count and direction are unchanged!!!!
    motor3.numSteps += discrepancy * motor3.gearRatio / motor3.stepResolution; // add the number of steps required to catch up or skip
    //numsteps gets updated but imagined angle doesnt...?
    }
    else Serial.println("not enough angle between current position and desired position to adjust during movement, waiting for movement to end");
    }
    else { // it's possible the check happens when the motor is at rest
    Serial.println("discrepancy is too high and motor is done moving, adjusting step number");
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
  Serial.print("Motor Angles: ");
  Serial.print(motor1.calcCurrentAngle());
  Serial.print(",");
  Serial.print(motor2.calcCurrentAngle());
  Serial.print(",");
  Serial.print(motor3.calcCurrentAngle());
  Serial.print(",");
  Serial.println(motor4.calcCurrentAngle()); // Serial.print(",");
  // Serial.print(motor5.calcCurrentAngle());Serial.print(",");
  // Serial.print(motor6.calcCurrentAngle());Serial.print(",");
}

void m3StepperInterrupt(void)
{
  static int nextInterval = STEPPER_PID_PERIOD; // how long until the next step is taken? indirectly controls speed
  if (motor3.isOpenLoop)
  {
    // open loop control
    // movementDone can be set elsewhere... so can numSteps
    if (!motor3.movementDone && motor3.stepCount < motor3.numSteps)
    {
      motor3.singleStep(motor3.rotationDirection); // direction was set beforehand
      motor3.stepCount++;
      if (motor3.hasRamping)
      {
        // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        nextInterval = stepIntervalArray[1] * 1000; // array is in ms not microseconds
        m3StepperTimer.update(nextInterval); // need to check if can call this inside the interrupt
      }
      // Serial.print(motor3.rotationDirection); Serial.println(" direction");
      // Serial.print(motor3.stepCount); Serial.println(" steps taken");
      // Serial.print(motor3.numSteps); Serial.println(" steps total");
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor3.stepCount = 0; // reset the counter
      motor3.movementDone = true;
      motor3.disablePower();
      // Serial.println("waiting for command");
    }
  }
  else
    if (!motor3.isOpenLoop)
    {
      // closed loop control
      ;
    }
}

void m4StepperInterrupt(void)
{
  static int nextInterval = STEPPER_PID_PERIOD;
  if (motor4.isOpenLoop)
  {
    // open loop control
    if (!motor4.movementDone && motor4.stepCount < motor4.numSteps)
    {
      motor4.singleStep(motor4.rotationDirection); // direction was set beforehand
      motor4.stepCount++;
      if (motor4.hasRamping)
      {
        // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        nextInterval = stepIntervalArray[1] * 1000; // array is in ms not microseconds
        m4StepperTimer.update(nextInterval); // need to check if can call this inside the interrupt
      }
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor4.stepCount = 0; // reset the counter
      motor4.movementDone = true;
      motor4.disablePower();
    }
  }
  else
    if (!motor4.isOpenLoop)
    {
      // closed loop control
      ;
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
        motor1.pidController.updatePID(motor1.currentAngle, motor1.desiredAngle);
        if (motor1.pidController.pidOutput == 0)
        {
          motor1.movementDone = true;
          motor1.stopRotation();
        }
        else
        {
          int motorSpeed = motor1.pidController.pidOutput * 255 / 100;
          motor1.calcDirection(motorSpeed); // does this work? it expects an angular error but at the end of the day...
          motor1.setVelocity(motor1.rotationDirection, motorSpeed);
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
        motor2.pidController.updatePID(motor2.currentAngle, motor2.desiredAngle);
        if (motor2.pidController.pidOutput == 0)
        {
          motor2.movementDone = true;
          motor2.stopRotation();
        }
        else
        {
          int motorSpeed = motor2.pidController.pidOutput * 255 / 100;
          motor2.calcDirection(motorSpeed); // does this work? it expects an angular error but at the end of the day...
          motor2.setVelocity(motor2.rotationDirection, motorSpeed);
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
        motor5.pidController.updatePID(motor5.currentAngle, motor5.desiredAngle);
        if (motor5.pidController.pidOutput == 0)
        {
          motor5.movementDone = true;
          motor5.stopRotation();
        }
        else
        {
          int motorSpeed = motor5.pidController.pidOutput * 255 / 100;
          motor5.calcDirection(motorSpeed); // does this work? it expects an angular error but at the end of the day...
          motor5.setVelocity(motor5.rotationDirection, motorSpeed);
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
        motor6.pidController.updatePID(motor6.currentAngle, motor6.desiredAngle);
        if (motor6.pidController.pidOutput == 0)
        {
          motor6.movementDone = true;
          motor6.stopRotation();
        }
        else
        {
          int motorSpeed = motor6.pidController.pidOutput * 255 / 100;
          motor6.calcDirection(motorSpeed); // does this work? it expects an angular error but at the end of the day...
          motor6.setVelocity(motor6.rotationDirection, motorSpeed);
        }
      }
      else
      {
        motor6.stopRotation();
      }
    }
}

void m1_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;
  Serial.print("m1 ");
  Serial.println(motor1.encoderCount);
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

void m2_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;
  // Serial.print("m2 "); Serial.println(motor2.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

void m3_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;
  Serial.println("m3 "); // Serial.println(motor3.encoderCount);
  // Serial.println(M3_ENCODER_PORT,BIN);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M3_ENCODER_PORT >> M3_ENCODER_SHIFT) & 0x03);
  motor3.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

void m4_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;
  Serial.print("m4 ");
  Serial.println(motor4.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M4_ENCODER_PORT >> M4_ENCODER_SHIFT) & 0x03);
  motor4.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}

/*
void m5_encoder_interrupt(void) {
static unsigned int oldEncoderState = 0;
Serial.print("m5 "); Serial.println(motor5.encoderCount);
oldEncoderState <<= 2;
oldEncoderState |= ((M5_ENCODER_PORT >> M5_ENCODER_SHIFT) & 0x03);
motor5.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}
void m6_encoder_interrupt(void) {
static unsigned int oldEncoderState = 0;
Serial.print("m6 "); Serial.println(motor6.encoderCount);
oldEncoderState <<= 2;
oldEncoderState |= ((M6_ENCODER_PORT >> M6_ENCODER_SHIFT) & 0x03);
motor6.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}
*/
void parseSerial(commandInfo& cmd)
{
  // check for emergency stop has precedence
  char * msgElem = strtok_r(restOfMessage, " ", & restOfMessage); // look for first element (first tag)
  if (String(msgElem) == "stop")
  {
    // msgElem is a char array so it's safer to convert to string first
    cmd.stopAllMotors = true;

#ifdef DEBUG_PARSING
    Serial.println("parsed emergency command to stop all motors");
#endif

  }
  // check for motor command
  else
    if (String(msgElem) == "motor")
    {
      // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (motor number)
      cmd.whichMotor = atoi(msgElem);

#ifdef DEBUG_PARSING
      Serial.print("parsed motor ");
      Serial.println(cmd.whichMotor);
#endif

      // check for motor stop command has precedence
      msgElem = strtok_r(NULL, " ", & restOfMessage); // find the next message element (direction tag)
      if (String(msgElem) == "stop")
      {
        // msgElem is a char array so it's safer to convert to string first
        cmd.stopSingleMotor = true;

#ifdef DEBUG_PARSING
        Serial.println("parsed request to stop single motor");
#endif

      }
      // check for angle command
      else
        if (String(msgElem) == "angle")
        {
          // msgElem is a char array so it's safer to convert to string first
          cmd.angleCommand = true;
          msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (desired angle value)
          cmd.whichAngle = atof(msgElem); // converts to float;

#ifdef DEBUG_PARSING
          Serial.print("parsed desired angle ");
          Serial.println(cmd.whichAngle);
#endif

        }
      // check for loop state command
      else
        if (String(msgElem) == "loop")
        {
          // msgElem is a char array so it's safer to convert to string first
          cmd.loopCommand = true;
          msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (desired angle value)
          if (String(msgElem) == "open")
          {
            cmd.loopState = OPEN_LOOP;

#ifdef DEBUG_PARSING
            Serial.println("parsed desired loop state ");
            Serial.println(cmd.loopState);
#endif

          }
          else
            if (String(msgElem) == "closed")
            {
              cmd.loopState = CLOSED_LOOP;

#ifdef DEBUG_PARSING
              Serial.println("parsed desired loop state ");
              Serial.println(cmd.loopState);
#endif

            }
        }
      // check for angle reset command
      else
        if (String(msgElem) == "reset")
        {
          // msgElem is a char array so it's safer to convert to string first
          cmd.resetCommand = true;
          msgElem = strtok_r(NULL, " ", & restOfMessage); // go to next msg element (desired angle value)
          if (String(msgElem) == "angle")
          {
            cmd.resetAngleValue = true;

#ifdef DEBUG_PARSING
            Serial.println("parsed request to reset angle value");
#endif

          }
          else
            if (String(msgElem) == "position")
            {
              cmd.resetJointPosition = true;

#ifdef DEBUG_PARSING
              Serial.println("parsed request to reset joint position");
#endif

            }
        }
    }
}

bool verifSerial(commandInfo cmd)
{
  if (cmd.stopAllMotors)
  {

#ifdef DEBUG_VERIFYING
    Serial.println("$S,Success: command to stop all motors verified");
#endif

    return true;
  }
  // 0 means there was an invalid command and therefore motors shouldn't be controlled
  else
    if (cmd.whichMotor > 0 && cmd.whichMotor <= RobotMotor::numMotors)
    {
      if (cmd.stopSingleMotor)
      {

#ifdef DEBUG_VERIFYING
        Serial.print("$S,Success: command to stop motor ");
        Serial.print(cmd.whichMotor);
        Serial.println(" verified");
#endif

        return true;
      }
      else
        if (cmd.angleCommand)
        {
          if (cmd.whichAngle < -720 || cmd.whichAngle > 720)
          {

#ifdef DEBUG_VERIFYING
            Serial.print("$E,Error: angle of ");
            Serial.print(cmd.whichAngle);
            Serial.print(" degrees invalid for motor ");
            Serial.println(cmd.whichMotor);
#endif

            return false;
          }
          else
          {

#ifdef DEBUG_VERIFYING
            Serial.print("$S,Success: command to move motor ");
            Serial.print(cmd.whichMotor);
            Serial.print(" ");
            Serial.print(cmd.whichAngle);
            Serial.println(" degrees verified");
#endif

            return true;
          }
        }
      else
        if (cmd.loopCommand)
        {
          if (cmd.loopState == OPEN_LOOP || cmd.loopState == CLOSED_LOOP)
          {

#ifdef DEBUG_VERIFYING
            Serial.print("$S,Success: command to set motor ");
            Serial.print(cmd.whichMotor);
            if (cmd.loopState == OPEN_LOOP)
              Serial.println(" to open loop verified");
            if (cmd.loopState == CLOSED_LOOP)
              Serial.println(" to closed loop verified");
#endif

            return true;
          }
          else
          {

#ifdef DEBUG_VERIFYING
            Serial.println("$E,Error: invalid loop state");
#endif

            return false;
          }
        }
      else
        if (cmd.resetCommand)
        {
          if (cmd.resetAngleValue || cmd.resetJointPosition)
          {

#ifdef DEBUG_VERIFYING
            Serial.print("$S,Success: command to reset motor ");
            Serial.print(cmd.whichMotor);
            if (cmd.resetAngleValue)
              Serial.println(" saved angle value verified");
            if (cmd.resetJointPosition)
              Serial.println(" physical joint position verified");
#endif

            return true;
          }
          else
          {

#ifdef DEBUG_VERIFYING
            Serial.println("$E,Error: invalid reset request");
#endif

            return false;
          }
        }
      else

#ifdef DEBUG_VERIFYING
      Serial.print("$E,Error: command for motor");
      Serial.print(cmd.whichMotor);
      Serial.println(" not recognized");
#endif

      return false;
    }
  else
    if
  (cmd.whichMotor < 0 || cmd.whichMotor >= RobotMotor::numMotors)
  {

#ifdef DEBUG_VERIFYING
    Serial.println("$E,Error: requested motor index out of bounds");
#endif

  }
  else
  {

#ifdef DEBUG_VERIFYING
    Serial.println("$E,Error: command not recognized");
#endif

    return false;
  }
}
