/*
  TODO:
  -suggestion 1 pinmode input, 2 is micros pwm instead of analogwrite (timer is bad?)
  -(always) implement error checking, remove parts later on if it's too slow (tatum doubts)
  -(always) clean up code, comment code
  -(done-ish) implement power management? sleep mode? marc says the power savings are negligible compared to other power savings
  -(done-ish) PID implemented for DC motor but need to adjust all variables, make setter functions
    -(done) update parsing function to take in angles
  -(done) leave serial.print debugging messages unless it really takes too much time (doubt)
  -(done) figure out how to solve the servo jitter. setting pins to INPUT probably fixes it
  -(depends on wiring) determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
  -(done-ish) drivers that accept 3.3v probably internally deal with it so that 3.3v is max speed
  -(done-ish) figure out how to send motor info from parsing function to budgeMotor etc

  -(done) implement budge function:
   -(done) parsing procedure can process direction, speed and time, which have defaults in budge()
   -(done) implement limit for max turns in right/left directions
   -(done) connect and test all motor types with budge()
   -(next) clean up budge code to make it easier to read and reduce repetition
   -(next) budge code doesn't work anymore with software interrupts, so I may just scrap it... or later i can figure out what's wrong with it

  -(next) clean up parsing code to make it easier to read and reduce repetition

  -(now) determine motor angles thru encoder interrupts
   -(done) all the pin and register definitions are based on the teensy, this would need to be changed for a different microcontroller
   -(done-ish) solve encoder direction change issue? not necessary
   -(issue) interrupt functions must be defined outside of classes...
   -(now) confirm all the pins will work with interrupts and not stepping on each other
   -(done-ish) encoder resolution and gear ratios determined for most motors but not m1,m5,m6
   -(done-ish) can determine motor angles for dc and steppers, not servos
   -(issue) not sure if encoder function reads all angles or if gear ratio/line count data is incorrect for PG188
   -(done) make sure there is  max/min angle limitation with encoders
   -(now) deal with overflow of encoderCount
   -(now) determine whether it's worth it to use the built in quadrature decoders

  -(next step) timers
   -systick is normally a heartbeat type thing
   -lptmr runs even on low power mode, maybe this should be heartbeat instead?
   -pit is used for intervaltimer objects, there are 4 and they work like interrupts
   -implement heartbeat after deciding on best timer for it
   -pwm: teensy has 6 16bit pwm timers and apparently 22 total pwm options:
    -currently using whatever is connected to the pwm pins for timing pwm
    -teensy pwm page mentions ftm and tpm timers which aren't mentioned in the page with other timesr
    -ftm+tpm has quadrature decoder?
    -tpm is 2-8 channel timer with pwm, 16bit counter, 2 channels for pwm

  -(next) simultaneous motor control with timers
   -(done) currently using IntervalTimers for software interrupts, may go lower level if necessary
   -(done-ish) software interrupts can take input from open loop control but need to refine everything and add closed loop control
    -(now) decide frequency of motor control loop: figure out estimated time for loop
    -(done) rewrote the motor control with timers for teensy, ensuring control for all motors
   -more comments in next comment block

  -(next next step) external interrupts for limit switches
   -(now) rewrite all the register bit variables to use teensy registers for limit switch interrupts
   -(waiting for the switches) incorporate limit switches for homing position on all motors
   -(later) implement homing function on boot

  -even more notes in josh notes.txt and in google drive
*/
/*
   perhaps pinsetup.h & pinsetup.cpp should be changed to motorsetup as it's also got angle limits and gear ratios
   technically i can use my motorarray shorthand to shorten the switch/case thing significantly
   gotta figure out the correspondence between direction pin's high/low and motor rotation direction
   set velocity should only set velocity and writing to pins happens outside in interrupt functions? or in dedicated function?

   I need to decide where movementDone is changed. does the pid decide that the movement is done and then the timer interrupts know not to turn motors?
   Right now there's a periodic check for STEPPERS that if there's a discrepancy then the stepper angle is adjusted. This is necessary because
   the steppers are controlled in open loop. So in this periodic check, it could also choose to disable the dc motor PID for example, eliminating the need
   to modify this variable which seems to not be related to the PID, and do it outside the pid object. that being the case, this loop is for stepper control,
   so it would also need to check dc control? given how often it checks it should be okay though........
   otherwise the pid WIll need to be inherited by the motor class and then implemented based on the motor type
   the advantage of this is that different motors have different types of speed output so it could be directly adapted inside the pid vs outside of it but
   that also may be bad coding practice.

   I also need to figure out where it's a good idea to disable interrupts so htat I don't read a value while it's being modified
*/

#include "PinSetup.h"

#include "RobotMotor.h"
#include "StepperMotor.h"
#include "DCMotor.h"
#include "ServoMotor.h"

#define STEPPER_PID_PERIOD 30*1000
#define DC_PID_PERIOD 40000 // 40ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 40000 // 40ms, because typical pwm signals have 20ms periods

#define STEPPER_CHECK_INTERVAL 2000
//#define STEPPER_CHECK_INTERVAL 250 // every 250ms check if the stepper is in the right spot

#define ENCODER_NVIC_PRIORITY 100 // should be higher priority tan most other things, quick calculations and happens very frequently
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4 // lower priority than motors but still important

/* serial */
#define BAUD_RATE 115200 // serial baud rate
#define SERIAL_PRINT_INTERVAL 1000 // how often should teensy send angle data
#define SERIAL_READ_TIMEOUT 50 // how often should the serial port be read
#define BUFFER_SIZE 100  // size of the buffer for the serial commands

char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent

/* parsing */
char *restOfMessage = serialBuffer; // used in strtok_r, which is the reentrant version of strtok
//int tempMotorVar; // checks the motor before giving the data to the struct below
//int tempSpeedVar; // checks the speed before giving the data to the struct below
//unsigned int tempTimeVar; // checks the time before giving the data to the struct below

struct budgeInfo { // info from parsing functionality is packaged and given to motor control functionality
  int whichMotor = 0;
  int whichDirection = 0;
  int whichSpeed = 0;
  unsigned int whichTime = 0;
  bool angleCommand = false;
  float whichAngle = 0.0;
} motorCommand, emptyMotorCommand; // emptyMotorCommand is used to reset the struct when the loop restarts

//quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states
const int encoderStates [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// instantiate motor objects here. only dcmotor currently supports interrupts
//StepperMotor motor1(M1_ENABLE_PIN, M1_DIR_PIN, M1_STEP_PIN, M1_STEP_RESOLUTION, FULL_STEP, M1_GEAR_RATIO);
DCMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO); // for new driver
//DCMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B); // sabertooth
DCMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO); // for new driver
StepperMotor motor3(M3_ENABLE_PIN, M3_DIR_PIN, M3_STEP_PIN, M3_STEP_RESOLUTION, FULL_STEP, M3_GEAR_RATIO);
StepperMotor motor4(M4_ENABLE_PIN, M4_DIR_PIN, M4_STEP_PIN, M4_STEP_RESOLUTION, FULL_STEP, M4_GEAR_RATIO);
ServoMotor motor5(M5_PWM_PIN, M5_GEAR_RATIO);
ServoMotor motor6(M6_PWM_PIN, M6_GEAR_RATIO);

IntervalTimer m3StepperTimer;
IntervalTimer m4StepperTimer;
IntervalTimer dcTimer;
IntervalTimer servoTimer;

elapsedMillis sinceStepperCheck;

//StepperMotor *m1 = &motor1;
DCMotor *m1 = &motor1;
DCMotor *m2 = &motor2;
StepperMotor *m3 = &motor3;
StepperMotor *m4 = &motor4;
ServoMotor *m5 = &motor5;
ServoMotor *m6 = &motor6;

RobotMotor *motorArray[] = {m1, m2, m3, m4, m5, m6};

void printMotorAngles();

void m1_encoder_interrupt(void);
void m2_encoder_interrupt(void);
void m3_encoder_interrupt(void);
void m4_encoder_interrupt(void);
//void m5_encoder_interrupt(void);
//void m6_encoder_interrupt(void);

void m3StepperInterrupt(void);
void m4StepperInterrupt(void);
void dcInterrupt(void);
void servoInterrupt(void);

void parseSerial(void);

void setup() {
  pinSetup();
  Serial.begin(BAUD_RATE); Serial.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms

  // to clean up: each motor with an encoder needs to setup that encoder
  motor1.attachEncoder(M1_ENCODER_A, M1_ENCODER_B, M1_ENCODER_PORT, M1_ENCODER_SHIFT, M1_ENCODER_RESOLUTION);
  motor2.attachEncoder(M2_ENCODER_A, M2_ENCODER_B, M2_ENCODER_PORT, M2_ENCODER_SHIFT, M2_ENCODER_RESOLUTION);
  motor3.attachEncoder(M3_ENCODER_A, M3_ENCODER_B, M3_ENCODER_PORT, M3_ENCODER_SHIFT, M3_ENCODER_RESOLUTION);
  motor4.attachEncoder(M4_ENCODER_A, M4_ENCODER_B, M4_ENCODER_PORT, M4_ENCODER_SHIFT, M4_ENCODER_RESOLUTION);

  // to clean up: each motor needs to attach 2 interrupts, which is a lot of lines of code
  attachInterrupt(motor1.encoderPinA, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor1.encoderPinB, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinA, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinB, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinA, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinB, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinA, m4_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinB, m4_encoder_interrupt, CHANGE);
  //attachInterrupt(motor5.encoderPinA, m5_encoder_interrupt, CHANGE);
  //attachInterrupt(motor5.encoderPinB, m5_encoder_interrupt, CHANGE);
  //attachInterrupt(motor6.encoderPinA, m6_encoder_interrupt, CHANGE);
  //attachInterrupt(motor6.encoderPinB, m6_encoder_interrupt, CHANGE);

  { // shaft angle tolerance setters
    //motor1.pidController.setAngleTolerance(1.8 * 3);
    motor1.pidController.setAngleTolerance(2.0);
    motor2.pidController.setAngleTolerance(2.0);
    motor3.pidController.setAngleTolerance(1.8 * 3);
    motor4.pidController.setAngleTolerance(1.8 * 3);
    motor5.pidController.setAngleTolerance(2.0);
    motor6.pidController.setAngleTolerance(2.0);
  }

  { // motor angle limit setters
    motor1.setAngleLimits(M1_MINIMUM_ANGLE, M1_MAXIMUM_ANGLE);
    motor2.setAngleLimits(M2_MINIMUM_ANGLE, M2_MAXIMUM_ANGLE);
    motor3.setAngleLimits(M3_MINIMUM_ANGLE, M3_MAXIMUM_ANGLE);
    motor4.setAngleLimits(M4_MINIMUM_ANGLE, M4_MAXIMUM_ANGLE);
    //motor5.setAngleLimits(M5_MINIMUM_ANGLE, M5_MAXIMUM_ANGLE); // this joint should be able to spin freely
    motor6.setAngleLimits(M6_MINIMUM_ANGLE, M6_MAXIMUM_ANGLE);
  }

  { // max (and min) speed setters
    motor1.pidController.setOutputLimits(0, 50);
    motor2.pidController.setOutputLimits(0, 50);
    motor3.pidController.setOutputLimits(0, 50);
    motor4.pidController.setOutputLimits(0, 50);
    motor5.pidController.setOutputLimits(0, 50);
    motor6.pidController.setOutputLimits(0, 50);
  }

  { // open loop setters
    motor1.isOpenLoop = true; motor1.hasRamping = false;
    motor1.openLoopSpeed = 50; // 50% speed
    motor1.openLoopGain = 5.0; // totally random guess, needs to be tested

    motor2.isOpenLoop = true; motor2.hasRamping = false;
    motor2.openLoopSpeed = 50; // 50% speed
    motor2.openLoopGain = 5.0; // totally random guess, needs to be tested

    motor3.isOpenLoop = true; motor3.hasRamping = false;
    motor3.openLoopSpeed = 50; // 50% speed
    motor3.openLoopGain = 5.0; // totally random guess, needs to be tested

    motor4.isOpenLoop = true; motor4.hasRamping = false;
    motor4.openLoopSpeed = 50; // 50% speed
    motor4.openLoopGain = 5.0; // totally random guess, needs to be tested

    motor5.isOpenLoop = true; motor5.hasRamping = false;
    motor5.openLoopSpeed = 50; // 50% speed
    motor5.openLoopGain = 5.0; // totally random guess, needs to be tested

    motor6.isOpenLoop = true; motor6.hasRamping = false;
    motor6.openLoopSpeed = 50; // 50% speed
    motor6.openLoopGain = 5.0; // totally random guess, needs to be tested

  }

  m3StepperTimer.begin(m3StepperInterrupt, STEPPER_PID_PERIOD); //1000ms
  m3StepperTimer.priority(MOTOR_NVIC_PRIORITY);
  m4StepperTimer.begin(m4StepperInterrupt, STEPPER_PID_PERIOD); //1000ms
  m4StepperTimer.priority(MOTOR_NVIC_PRIORITY);
  dcTimer.begin(dcInterrupt, DC_PID_PERIOD); //need to choose a period... went with 20ms because that's typical pwm period for servos...
  dcTimer.priority(MOTOR_NVIC_PRIORITY);
  servoTimer.begin(servoInterrupt, SERVO_PID_PERIOD); //need to choose a period... went with 20ms because that's typical pwm period for servos...
  servoTimer.priority(MOTOR_NVIC_PRIORITY);

  sinceAnglePrint = 0;
  sinceStepperCheck = 0;
}

void loop() {

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.println("=======================================================");
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received
    parseSerial(); // goes through the message and puts the appropriate data into motorCommand struct
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    restOfMessage = serialBuffer; // reset pointer
  }

  if (motorCommand.whichMotor > 0) {
    if (motorCommand.angleCommand) {
      Serial.print("motor "); Serial.print(motorCommand.whichMotor);
      Serial.print(" desired angle (degrees) is: "); Serial.println(motorCommand.whichAngle);
      Serial.println("=======================================================");
      switch (motorCommand.whichMotor) { // move a motor based on which one was commanded
        case MOTOR1:
          if (motorCommand.whichAngle > motor1.minimumAngle && motorCommand.whichAngle < motor1.maximumAngle) {
            //motor1.movementDone = false;
            //motor1.enablePower();
            // If I do the code like this, it means that once the motor achieves the correct position,
            // it will stop and never recorrect until a new angle request is sent.
            // Also, technically updatePID should only be called in the stepper interrupt.
            // Well, technically it should only be used for the DC motor....
            motor1.desiredAngle = motorCommand.whichAngle;
            motor1.pidController.updatePID(motor1.currentAngle, motor1.desiredAngle);
          }
          else Serial.println("$E,Alert: requested angle is not within angle limits.");
          break;
        case MOTOR2:
          if (motor2.setDesiredAngle(motorCommand.whichAngle)) {
            if (motor2.isOpenLoop) {
              motor2.openLoopError = motor2.desiredAngle;

              // determine the direction
              if (motor2.openLoopError >= 0) motor2.openLoopDirection = 1;
              else motor2.openLoopDirection = -1;

              motor2.calcTurningDuration();
              motor2.timeCount = 0;
            }
            else {
              // actually shouldn't the pid only be updated in the timer interrupt?
              //motor2.calcCurrentAngle(); // find the angle difference
              //motor2.pidController.updatePID(motor2.currentAngle, motor2.desiredAngle);
            }
            motor2.movementDone = false; // this flag being false lets the timer interrupt control the dc motor speed
          }
          else Serial.println("$E,Alert: requested angle is not within angle limits.");
          break;
        case MOTOR3:
          // motor3.setDesiredAngle(motorCommand.whichAngle); // setDesiredAngle(float angle) { if (angle < this.minimumAgle || angle > this.maximumAngle) { throw new IllegalArgumentException("You retard")}}
          if (motorCommand.whichAngle > motor3.minimumAngle && motorCommand.whichAngle < motor3.maximumAngle) {
            //((StepperMotor *)motorArray[motorCommand.whichMotor-1])->desiredAngle = motorCommand.whichAngle; // I hate this
            /*
               I wanted to have an array for motor1,motor2, etc and then just use commands like the above,
               but in order for that to work i still need to use that type caster beforehand, which defeats hte purpose
               of the array. the point is to not have to have cases but i still need them...
            */
            motor3.desiredAngle = motorCommand.whichAngle; // set the desired angle based on the command
            motor3.openLoopError = motor3.desiredAngle - motor3.calcCurrentAngle(); // find the angle difference

            // determine the direction
            if (motor3.openLoopError >= 0) motor3.openLoopDirection = 1;
            else motor3.openLoopDirection = -1;

            // if the error is big enough to justify movement
            // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
            if ( fabs(motor3.openLoopError) > motor3.pidController.angleTolerance * motor3.gearRatioReciprocal) {
              motor3.numSteps = fabs(motor3.openLoopError) * motor3.gearRatio
                                / motor3.stepResolution; // calculate the number of steps to take
              motor3.enablePower(); // give power to the stepper finally
              motor3.movementDone = false; // this flag being false lets the timer interrupt move the stepper
            }
            else Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
            //motor3.pidController.updatePID(motor3.currentAngle, motor3.desiredAngle);
          }
          else Serial.println("$E,Alert: requested angle is not within angle limits.");
          break;
        case MOTOR4:
          if (motorCommand.whichAngle > motor4.minimumAngle && motorCommand.whichAngle < motor4.maximumAngle) {
            motor4.desiredAngle = motorCommand.whichAngle; // set the desired angle based on the command
            motor4.openLoopError = motor4.desiredAngle - motor4.calcCurrentAngle(); // find the angle difference

            // determine the direction
            if (motor4.openLoopError >= 0) motor4.openLoopDirection = 1;
            else motor4.openLoopDirection = -1;

            // if the error is big enough to justify movement
            // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
            if ( fabs(motor4.openLoopError) > motor4.pidController.angleTolerance * motor4.gearRatioReciprocal) {
              motor4.numSteps = fabs(motor4.openLoopError) * motor4.gearRatio
                                / motor4.stepResolution; // calculate the number of steps to take
              motor4.enablePower(); // give power to the stepper finally
              motor4.movementDone = false; // this flag being false lets the timer interrupt move the stepper
            }
            else Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
            //motor4.pidController.updatePID(motor4.currentAngle, motor4.desiredAngle);
          }
          else Serial.println("$E,Alert: requested angle is not within angle limits.");
          break;
        case MOTOR5:
          // this motor has no max or min angle because it must be able to spin like a screwdriver
          //motor5.pidController.updatePID(motor5.currentAngle, motor5.desiredAngle);
          motor5.desiredAngle = motorCommand.whichAngle; // set the desired angle based on the command
          motor5.openLoopError = motor5.desiredAngle; //- motor5.calcCurrentAngle(); // find the angle difference

          // determine the direction
          if (motor5.openLoopError >= 0) motor5.openLoopDirection = 1;
          else motor5.openLoopDirection = -1;

          motor5.openLoopSpeed = 50; // 50% speed
          motor5.timeCount = 0;

          // if the error is big enough to justify movement
          // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
          if ( fabs(motor5.openLoopError) > motor5.pidController.angleTolerance * motor5.gearRatioReciprocal) {
            motor5.numMillis = (fabs(motor5.openLoopError) * motor5.gearRatio / motor5.openLoopSpeed)
                               * 1000.0 * motor5.openLoopGain; // calculate how long to turn for
            motor5.movementDone = false; // this flag being false lets the timer interrupt move the stepper
            //Serial.println(motor5.numMillis);
          }
          else Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
          //motor5.pidController.updatePID(motor5.currentAngle, motor5.desiredAngle);
          break;
        case MOTOR6:
          if (motorCommand.whichAngle > motor6.minimumAngle && motorCommand.whichAngle < motor6.maximumAngle) {
            //motor5.pidController.updatePID(motor5.currentAngle, motor5.desiredAngle);
            motor6.desiredAngle = motorCommand.whichAngle; // set the desired angle based on the command
            motor6.openLoopError = motor6.desiredAngle; //- motor6.calcCurrentAngle(); // find the angle difference

            // determine the direction
            if (motor6.openLoopError >= 0) motor6.openLoopDirection = 1;
            else motor6.openLoopDirection = -1;

            motor6.openLoopSpeed = 50; // 50% speed
            motor6.timeCount = 0;

            // if the error is big enough to justify movement
            // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
            if ( fabs(motor6.openLoopError)  > motor6.pidController.angleTolerance * motor6.gearRatioReciprocal) {
              motor6.numMillis = (fabs(motor6.openLoopError) * motor6.gearRatio / motor6.openLoopSpeed)
                                 * 1000.0 * motor6.openLoopGain; // calculate how long to turn for
              motor6.movementDone = false; // this flag being false lets the timer interrupt move the stepper
              //Serial.println(motor6.numMillis);
            }
            else Serial.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
            //motor6.pidController.updatePID(motor6.currentAngle, motor6.desiredAngle);
          }
          else Serial.println("$E,Alert: requested angle is not within angle limits.");
          break;
      }
    }
    else if ( (motorCommand.whichDirection == 1 || motorCommand.whichDirection == -1) && motorCommand.whichSpeed > 0
              && motorCommand.whichTime > 0) {
      Serial.print("motor "); Serial.print(motorCommand.whichMotor); Serial.println(" to move");
      Serial.println("=======================================================");

      // activate budge command for appropriate motor
      motorArray[motorCommand.whichMotor - 1]->budge(motorCommand.whichDirection, motorCommand.whichSpeed, motorCommand.whichTime);
    }
    else Serial.println("$E,Error: bad motor command");
  }
  motorCommand = emptyMotorCommand; // reset motorCommand so the microcontroller doesn't try to move a motor next loop


  if (sinceStepperCheck >= STEPPER_CHECK_INTERVAL) {
    /* this code could (should?) also disable power */
    //if (motor1.movementDone) motor1.disablePower();
    //if (motor3.movementDone) motor3.disablePower();
    //if (motor4.movementDone) motor4.disablePower();
    /* this code could (should?) determine when servo/dc motor movement is done */
    /*
      if ( fabs(motor2.desiredAngle - motor2.calcCurrentAngle() ) < motor2.pidController.angleTolerance){
      motor2.movementDone = true;
      }
    */
    /*
      if ( fabs(motor5.desiredAngle - motor5.calcCurrentAngle() ) < motor5.pidController.angleTolerance){
      motor5.movementDone = true;
      }
      if ( fabs(motor6.desiredAngle - motor6.calcCurrentAngle() ) < motor6.pidController.angleTolerance){
      motor6.movementDone = true;
      }
    */

    // all of this code should probably go into a function called "calculate motor steps" or something...
    // this code is very similar to what happens above when it decides which motor should turn after receiving a command
    // this code also assumes that the correct amount of steps will take it to the right spot
    // this means that it doesn't account for faulty angle calculations from the encoder or the motor resolution...

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

    //Serial.print(imaginedAngle); Serial.println(" imagined angle");
    //Serial.print(actualAngle); Serial.println(" actual angle");
    //Serial.print(imaginedRemainingAngle); Serial.println(" imagined remaining angle");
    //Serial.print(actualRemainingAngle); Serial.println(" actual remaining angle");
    //Serial.print(discrepancy); Serial.println(" degrees behind expected position");

    /*

        // the stepper interrupt could occur during this calculation, so maybe there should be a different angle tolerance here
        // that said at the moment it's 2 degrees which is bigger than the max step angle of the motor
        // keep in mind that 2 degrees for the joint is different from 2 degrees for the motor shaft
        if (fabs(discrepancy) * motor3.gearRatio > motor3.pidController.angleTolerance) {
          Serial.println("discrepancy is too high and motor is moving, adjusting step number");
          // it's possible the check happens during movement, but there needs to be ample distance to move
          if (!motor3.movementDone) {
          // if actualRemainingAngle is negative it means the arm moved way further than it should have
            if(actualRemainingAngle<0) motor3.movementDone=true; // abort
            else if(actualRemainingAngle > motor3.pidController.angleTolerance){
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
            if (discrepancy >= 0) motor3.openLoopDirection = 1;
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
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) {
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }

}

void printMotorAngles() {
  Serial.print("Motor Angles: ");
  Serial.print(motor1.calcCurrentAngle()); Serial.print(",");
  Serial.print(motor2.calcCurrentAngle()); Serial.print(",");
  Serial.print(motor3.calcCurrentAngle()); Serial.print(",");
  Serial.println(motor4.calcCurrentAngle());//Serial.print(",");
  //Serial.print(motor5.calcCurrentAngle());Serial.print(",");
  //Serial.print(motor6.calcCurrentAngle());Serial.print(",");
}

/*
  // for multiple steppers in same timer interrupt (incomplete, complicated to implement)
  void stepperInterrupt(void) {
  static int nextInterval = 0;
  int i = 3;
  // code to decide which motor to turn goes here, or code just turns all motors
  if (!stepperArray[i - 1].movementDone) {
    // code to decide how fast the motor will turn and in which direction
    int dir = CLOCKWISE;
    nextInterval = 25000;
    //set a minimum step period somewhere
    //nextInterval = stepperArray[i-1].pidController.updatePID(stepperArray[i-1].currentAngle, stepperArray[i-1].desiredAngle);
    stepperArray[i - 1].singleStep(dir);
    stepperTimer.update(nextInterval);
  }
  }
*/

void m3StepperInterrupt(void) {
  static int nextInterval = STEPPER_PID_PERIOD;
  if (motor3.isOpenLoop) { // open loop control
    // movementDone can be set elsewhere... so can numSteps
    if (!motor3.movementDone && motor3.stepCount < motor3.numSteps) {
      motor3.singleStep(motor3.openLoopDirection); // direction was set beforehand
      motor3.stepCount++;
      if (motor3.hasRamping) { // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        nextInterval = stepIntervalArray[1] * 1000; // array is in ms not microseconds
        m3StepperTimer.update(nextInterval); // need to check if can call this inside the interrupt
      }
      //Serial.print(motor3.openLoopDirection); Serial.println(" direction");
      //Serial.print(motor3.stepCount); Serial.println(" steps taken");
      //Serial.print(motor3.numSteps); Serial.println(" steps total");
    }
    else { // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor3.stepCount = 0; // reset the counter
      //motor3.movementDone = true;
      motor3.movementDone = true;
      motor3.disablePower();
      //Serial.println("waiting for command");
    }
  }
  else { // closed loop control
    ;
  }
}

void m4StepperInterrupt(void) {
  static int nextInterval = STEPPER_PID_PERIOD;
  if (motor4.isOpenLoop) { // open loop control
    if (!motor4.movementDone && motor4.stepCount < motor4.numSteps) {
      motor4.singleStep(motor4.openLoopDirection); // direction was set beforehand
      motor4.stepCount++;
      if (motor4.hasRamping) { // if speed ramping is enabled
        // following code has array index that should be incremented each interrupt
        nextInterval = stepIntervalArray[1] * 1000; // array is in ms not microseconds
        m4StepperTimer.update(nextInterval); // need to check if can call this inside the interrupt
      }
    }
    else { // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor4.stepCount = 0; // reset the counter
      motor4.movementDone = true;
      motor4.disablePower();
    }
  }
  else { // closed loop control
    ;
  }
}

void dcInterrupt(void) {
  // code to decide which motor to turn goes here, or code just turns all motors
  // check motor 2 for dc motor control planning
  // motor 1
  if (motor1.isOpenLoop) { // open loop control
    if (!motor1.movementDone && motor1.timeCount < motor1.numMillis) {
      motor1.setVelocity(motor1.openLoopDirection, motor1.openLoopSpeed);
      Serial.print("Beep");
    }
    else {
      motor1.movementDone = true;
      motor1.stopRotation();
    }
  }
  else { // closed loop control, incomplete
    ;
  }
  // motor 2
  if (motor2.isOpenLoop) { // open loop control
    if (!motor2.movementDone && motor2.timeCount < motor2.numMillis) {
      motor2.setVelocity(motor2.openLoopDirection, motor2.openLoopSpeed);
    }
    else {
      motor2.movementDone = true;
      motor2.stopRotation();
    }
    // would be nice to have some kind of check for the above functions so the command only runs if there's been a change
    // e.g. movementDone changed or the speed or numMillis changed
  }
  else { // closed loop control, incomplete
    if (!motor2.movementDone) {
      // rethink the PID? needs to set a direction AND a speed
      //motor2.calcCurrentAngle();
      //motor2.pidController.updatePID(motor2.currentAngle, motor2.desiredAngle);
      // 255 is arbitrary value... some conversion probably required between pid output and motor speed input
      //int motorSpeed = motor2.pidController.pidOutput * 255;
      //motor2.setVelocity(dir, motorSpeed);
    }
  }
}
// final implementation would have the PID being called in these interrupts

void servoInterrupt(void) {
  // movementDone can be set elsewhere... so can numSteps
  // motor 5
  if (motor5.isOpenLoop) { // open loop control
    if (!motor5.movementDone && motor5.timeCount < motor5.numMillis) {
      //Serial.println("command being processed");
      motor5.setVelocity(motor5.openLoopDirection, motor5.openLoopSpeed);
    }
    else { // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor5.movementDone = true;
      motor5.stopRotation();
    }
  }
  else { // closed loop control
    ;
  }
  // motor 6
  if (motor6.isOpenLoop) { // open loop control
    if (!motor6.movementDone && motor6.timeCount < motor6.numMillis) {
      motor6.setVelocity(motor6.openLoopDirection, motor6.openLoopSpeed);
    }
    else { // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      motor6.movementDone = true;
      motor6.stopRotation();
    }
  }
  else { // closed loop control
    ;
  }
}

/*
  // for now these interrupts are open loop only, unless encoders are integrated into the servos
  void m5ServoInterrupt(void) {
  // movementDone can be set elsewhere... so can numSteps
  if (!motor5.movementDone && motor5.timeCount < motor5.numMillis) {
    //Serial.println("command being processed");
    motor5.setVelocity(motor5.openLoopDirection, motor5.openLoopSpeed);
  }
  else { // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
    motor5.movementDone = true;
    motor5.stopRotation();
  }
  }

  void m6ServoInterrupt(void) {
    // movementDone can be set elsewhere... so can numSteps
  if (!motor6.movementDone && motor6.timeCount < motor6.numMillis) {
    //Serial.println("command being processed");
    motor6.setVelocity(motor6.openLoopDirection, motor6.openLoopSpeed);
  }
  else { // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
    motor6.movementDone = true;
    motor6.stopRotation();
  }
  }
*/
// if these don't register fast enough, use global volatile long encoderCount variables per motor instead of acccessing objects twice
void m1_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m1 "); Serial.println(motor1.encoderCount);
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
void m2_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m2 "); Serial.println(motor2.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}
void m3_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.println("m3 "); //Serial.println(motor3.encoderCount);
  //Serial.println(M3_ENCODER_PORT,BIN);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M3_ENCODER_PORT >> M3_ENCODER_SHIFT) & 0x03);
  motor3.encoderCount += encoderStates[(oldEncoderState & 0x0F)];
}
void m4_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m4 "); Serial.println(motor4.encoderCount);
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

void parseSerial(void) {
  char* msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // look for first element (first tag)
  if (String(msgElem) == "motor") { // msgElem is a char array so it's safer to convert to string first
    msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (motor number)
    int tempMotorVar = atoi(msgElem);
    // currently uses motor1's numMotors variable which is shared by all RoverMotor objects and children. need better implementation
    if (tempMotorVar > 0 && tempMotorVar <= RobotMotor::numMotors) {
      motorCommand.whichMotor = tempMotorVar;
      Serial.print("parsed motor "); Serial.println(motorCommand.whichMotor);
    }
    else Serial.println("motor does not exist");
    msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (direction tag)
    if (String(msgElem) == "angle") { // msgElem is a char array so it's safer to convert to string first
      motorCommand.angleCommand = true;
      msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (desired angle value)
      float tempAngleVar = atof(msgElem); // converts to float
      if (tempAngleVar > -720.0 && tempAngleVar < 720.0) {
        motorCommand.whichAngle = tempAngleVar;
        Serial.print("parsed desired angle "); Serial.println(motorCommand.whichAngle);
      }
      else Serial.println("angle is out of bounds");
    }
    else if (String(msgElem) == "direction") { // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (direction value)
      //float tempDirVar = atof(msgElem); // converts to float
      switch (*msgElem) { // determines motor direction
        case '0': // arbitrarily (for now) decided 0 is clockwise
          motorCommand.whichDirection = CLOCKWISE;
          Serial.println("parsed direction clockwise");
          break;
        case '1': // arbitrarily (for now) decided 1 is counter-clockwise
          motorCommand.whichDirection = COUNTER_CLOCKWISE;
          Serial.println("parsed direction counter-clockwise");
          break;
      }
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (speed tag)
      if (String(msgElem) == "speed") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (integer representing speed level)
        int tempSpeedVar = atoi(msgElem); // converts to int
        if (tempSpeedVar <= MAX_SPEED - 1) { // make sure the speed is below 4, change this later to expect values 1-4 instead of 0-3
          motorCommand.whichSpeed = tempSpeedVar + 1; // set the actual speed, enum starts with 1
          Serial.print("parsed speed level: "); Serial.println(motorCommand.whichSpeed);
        }
      }
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (time tag)
      if (String(msgElem) == "time") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (time in seconds)
        unsigned int tempTimeVar = atoi(msgElem); // converts to int
        if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
          motorCommand.whichTime = tempTimeVar;
          Serial.print("parsed time interval "); Serial.print(motorCommand.whichTime); Serial.println("ms");
        }
      }
    }
  }
}
/*
  // takes the key (speed, time, etc) and updates?outputs? the next value
  // this one is for unsigned ints, would need to define separate ones for other data types?
  parseKey(String key, unsigned int valueOut, String delimeter, unsigned int minVal, unsigned int maxVal) {
  char *msgElem = strtok_r(NULL, delimeter, &restOfMessage); // find the next message element
  if (String(msgElem) == key) { // msgElem is a char array so it's safer to convert to string first
    msgElem = strtok_r(NULL, delimeter, &restOfMessage); // find the next message element (time in seconds)
    unsigned int tempVar = atoi(msgElem); // converts to int
    //if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
    if (tempVar <= maxVal && tempVar >= minVal) {
      //motorCommand.whichTime = tempTimeVar;
      valueOut = tempVar;
      Serial.print("parsed time interval "); Serial.print(motorCommand.whichTime); Serial.println("ms");
    }
  }
  }
*/
