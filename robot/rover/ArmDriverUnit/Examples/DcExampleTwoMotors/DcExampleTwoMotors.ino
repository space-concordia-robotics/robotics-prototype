/*
This is an example sketch using the unfinished motor libraries. It allows the user
to control two DC motors in open or closed loop over the serial port. In the future,
some of the header files used in this code should be turned into libraries.
*/

/* still in idea phase */
#define DEVEL1_MODE 1 // sends messages with Serial, everything unlocked
// #define DEVEL2_MODE 2 // sends messages with Serial1, everything unlocked
#define DEBUG_MAIN 10 // debug messages during main loop
#define DEBUG_PARSING 11 // debug messages during parsing function
#define DEBUG_VERIFYING 12 // debug messages during verification function
// #define DEBUG_ENCODERS 13 // debug messages during encoder interrupts
#define DEBUG_SWITCHES 14 // debug messages during limit switch interrupts
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

// includes must come after the above UART_PORT definition as it's used in other files.
// perhaps it should be placed in pinsetup.h (which has to be renamed anyway)...
#include "PinSetup.h"
#include "Parser.h"
// #include "Notes.h" // holds todo info
// #include "Ideas.h" // holds bits of code that haven't been implemented
#include "PidController.h"
#include "RobotMotor.h"
#include "DcMotor.h"
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
/* parsing */
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
/* motors */
// quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states
const int encoderStates[16] =
{
  0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
};

// instantiate motor objects here:
DcMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO); // for cytron
// DcMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B); // sabertooth
DcMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO); // for cytron
// motor array prep work: making pointers to motor objects
DcMotor * m1 = & motor1;
DcMotor * m2 = & motor2;
// I can use this instead of switch/case statements by doing motorArray[motornumber]->attribute
RobotMotor * motorArray[] =
{
  m1, m2
};

// instantiate timers here:
IntervalTimer dcTimer; // motors 1&2
// these are a nicer way of timing events than using millis()
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent
/* function declarations */
void printMotorAngles(); // sends all motor angles over serial
// all interrupt service routines (ISRs) must be global functions to work
// declare encoder interrupt service routines

#ifdef M1_ENCODER_PORT
void m1_encoder_interrupt(void);
#endif
#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void);
#endif
// declare limit switch interrupt service routines
void m1CwISR(void);
void m1CcwISR(void);
void m2FlexISR(void);
void m2ExtendISR(void);
// declare timer interrupt service routines, where the motors actually get controlled
void dcInterrupt(void); // manages motors 1&2
/* Teensy setup */
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
  // prepare and attach limit switch ISRs

#if defined(LIM_SWITCH_FALL)
  #define LIM_SWITCH_DIR FALLING
#elif defined(LIM_SWITCH_RISE)
  #define LIM_SWITCH_DIR RISING
#endif

  motor1.attachLimitSwitches('c', M1_LIMIT_SW_CW, M1_LIMIT_SW_CCW);
   motor2.attachLimitSwitches('f', M2_LIMIT_SW_FLEX, M2_LIMIT_SW_EXTEND);
  attachInterrupt(motor1.limSwitchCw, m1CwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor1.limSwitchCcw, m1CcwISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchFlex, m2FlexISR, LIM_SWITCH_DIR);
  attachInterrupt(motor2.limSwitchExtend, m2ExtendISR, LIM_SWITCH_DIR);
  // set motor shaft angle tolerances
  // motor1.pidController.setJointAngleTolerance(1.8 * 3*motor1.gearRatioReciprocal); // if it was a stepper
  motor1.pidController.setJointAngleTolerance(2.0 * motor1.gearRatioReciprocal); // randomly chosen for dc
  motor2.pidController.setJointAngleTolerance(2.0 * motor2.gearRatioReciprocal);
  // set motor joint angle limits
  motor1.setAngleLimits(M1_MINIMUM_ANGLE, M1_MAXIMUM_ANGLE);
  motor2.setAngleLimits(M2_MINIMUM_ANGLE, M2_MAXIMUM_ANGLE);
  // set max and min closed loop speeds (in percentage), I limit it to 50% for safety
  // Abtin thinks 50% should be a hard limit that can't be modified this easily
  motor1.pidController.setOutputLimits(-50, 50, 5.0);
  // motor2.pidController.setOutputLimits(-100, 100, 5.0);
  motor2.pidController.setOutputLimits(-50, 50, 5.0);
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
  // activate the timer interrupts
  dcTimer.begin(dcInterrupt, DC_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  dcTimer.priority(MOTOR_NVIC_PRIORITY);
  // reset the elapsedMillis variables so that they're fresh upon entering the loop()
  sinceAnglePrint = 0;
}

/* main code loop */
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
    Parser.parseCommand(motorCommand, serialBuffer);
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    if (!Parser.verifCommand(motorCommand))
    {
      // do nothing in particular
      // UART_PORT.println("$E,Error: verification failed");
    }
    else
    {
      UART_PORT.println(messageBar);
      // emergency stop takes precedence
      if (motorCommand.stopAllMotors)
      {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
          motorArray[i] -> stopRotation();
        }
        UART_PORT.println("all motors stopped because of emergency stop");
      }
      else
      {
        // stopping a single motor takes precedence
        if (motorCommand.stopSingleMotor)
        {
          motorArray[motorCommand.whichMotor - 1] -> stopRotation();
          UART_PORT.print("stopped motor ");
          UART_PORT.println(motorCommand.whichMotor);
        }
        // make motors move simultaneously
        else
          if (motorCommand.multiMove)
          {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
              if (motorCommand.motorsToMove[i])
              {
                UART_PORT.print("motor ");
                UART_PORT.print(i + 1);
                UART_PORT.print(" desired angle (degrees) is: ");
                UART_PORT.println(motorCommand.anglesToReach[i]);
              }
              else
              {
                UART_PORT.print("motor ");
                UART_PORT.print(i + 1);
                UART_PORT.println(" will not change course");
              }
            }
            UART_PORT.println(messageBar);
            bool motorsCanMove = true;
            for (int i = 0; i < NUM_MOTORS; i++)
            {
              if (!(motorArray[i] -> withinJointAngleLimits(motorCommand.anglesToReach[i])))
              {
                motorsCanMove = false;
                UART_PORT.print("$E,Alert: requested motor ");
                UART_PORT.print(i + 1);
                UART_PORT.println(" angle is not within angle limits.");
              }
            }
            if (!motorsCanMove)
            {
              UART_PORT.println("$E,Error: one or many angles are invalid, arm will not move");
            }
            else
            {
              UART_PORT.println("$S,Success: all angles are valid, arm to move");
              if (motorCommand.motorsToMove[0])
              {
                // this motor can swivel but has limits so it doesn't hit anything
                if (motor1.setDesiredAngle(motorCommand.anglesToReach[0]))
                {
                  // this method returns true if the command is within joint angle limits
                  if (motor1.isOpenLoop)
                  {
                    motor1.calcCurrentAngle();
                    motor1.openLoopError = motor1.getDesiredAngle() - motor1.getCurrentAngle(); // find the angle difference
                    motor1.calcDirection(motor1.openLoopError); // determine rotation direction and save the value
                    // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
                    if (motor1.calcTurningDuration(motor1.openLoopError))
                    // returns false if the open loop error is too small
                    {
                      motor1.timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
                      motor1.movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
                      UART_PORT.print("$S,Success: motor ");
                      UART_PORT.print(1);
                      UART_PORT.print(" to turn for ");
                      UART_PORT.print(motor1.numMillis);
                      UART_PORT.println(" milliseconds");
                    }
                    else
                    {
                      UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                    }
                  }
                  else
                    if (!motor1.isOpenLoop)
                    {
                      // all the heavy lifting for closed loop control is done in the timer interrupt
                      motor1.movementDone = false;
                    }
                }
              }
              if (motorCommand.motorsToMove[1])
              {
                if (motor2.setDesiredAngle(motorCommand.anglesToReach[1]))
                {
                  if (motor2.isOpenLoop)
                  {
                    motor2.calcCurrentAngle();
                    motor2.openLoopError = motor2.getDesiredAngle() - motor2.getCurrentAngle();
                    motor2.calcDirection(motor2.openLoopError);
                    if (motor2.calcTurningDuration(motor2.openLoopError))
                    {
                      motor2.timeCount = 0;
                      motor2.movementDone = false;
                      UART_PORT.print("$S,Success: motor ");
                      UART_PORT.print(2);
                      UART_PORT.print(" to turn for ");
                      UART_PORT.print(motor2.numMillis);
                      UART_PORT.println(" milliseconds");
                    }
                    else
                    {
                      UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                    }
                  }
                  else
                    if (!motor2.isOpenLoop)
                    {
                      motor2.movementDone = false;
                    }
                }
                else
                {
                  UART_PORT.println("$E,Alert: requested angle is not within angle limits.");
                }
              }
            }
          }
        // set loop states for appropriate motor
        else
          if (motorCommand.loopCommand)
          {
            if (motorCommand.loopState == OPEN_LOOP)
            {
              motorArray[motorCommand.whichMotor - 1] -> isOpenLoop = true;
              UART_PORT.print("motor ");
              UART_PORT.print(motorCommand.whichMotor);
              UART_PORT.println(" is open loop");
            }
            else
              if (motorCommand.loopState == CLOSED_LOOP)
              {
                if (motorArray[motorCommand.whichMotor - 1] -> hasEncoder)
                {
                  motorArray[motorCommand.whichMotor - 1] -> isOpenLoop = false;
                }
                else
                {
                  UART_PORT.println("$E,Alert: cannot use closed loop if motor has no encoder.");
                }
              }
          }
        // reset the motor angle's variable or actually control the motor to reset it to neutral position
        else
          if (motorCommand.resetCommand)
          {
            if (motorCommand.resetAngleValue)
            {
              motorArray[motorCommand.whichMotor - 1] -> setImaginedAngle(0.0);
              UART_PORT.print("reset angle value of motor ");
              UART_PORT.println(motorCommand.whichMotor);
            }
            else
              if (motorCommand.resetJointPosition)
              {
                ; // for later
              }
          }
        // change the direction modifier to swap rotation direction in the case of backwards wiring
        else
          if (motorCommand.switchDir)
          {
            motorArray[motorCommand.whichMotor - 1] -> switchDirectionLogic();
            UART_PORT.print("direction modifier is now ");
            UART_PORT.println(motorArray[motorCommand.whichMotor - 1] -> getDirectionLogic());
          }
        else
          UART_PORT.println("$E,Error: bad motor command");
      }
    }
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
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motorArray[i] -> calcCurrentAngle();
      if (motorArray[i] -> isOpenLoop)
      {
        UART_PORT.print(motorArray[i] -> getImaginedAngle());
      }
      else
      {
        UART_PORT.print(motorArray[i] -> getCurrentAngle());
      }
      UART_PORT.print(", ");
  }
  UART_PORT.println("");
}

void dcInterrupt(void)
{
  // movementDone can be set elsewhere... so can numMillis, openLoopSpeed and rotationDirection (in open loop control)
  if (motor1.isOpenLoop)
  {
    // open loop control
    if (!motor1.movementDone && motor1.timeCount <= motor1.numMillis)
    {
      // calculates the pwm to send to the motor and makes it move
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
        // determine the speed of the motor until the next interrupt
        float output = motor1.pidController.updatePID(motor1.getCurrentAngle(), motor1.getDesiredAngle());
        if (output == 0)
        {
          motor1.movementDone = true;
          motor1.stopRotation();
        }
        else
        {
          int dir = motor1.calcDirection(output);
          // calculates the pwm to send to the motor and makes it move
          motor1.setVelocity(dir, output);
        }
      }
      else
      {
        motor1.stopRotation();
      }
    }
    if (motor2.isOpenLoop)
  {
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
  else
    if (!motor2.isOpenLoop)
    {
      if (!motor2.movementDone)
      {
        motor2.calcCurrentAngle();
        float output = motor2.pidController.updatePID(motor2.getCurrentAngle(), motor2.getDesiredAngle());
        if (output == 0)
        {
          motor2.movementDone = true;
          motor2.stopRotation();
        }
        else
        {
          int dir = motor2.calcDirection(output);
          motor2.setVelocity(dir, output);
        }
      }
      else
      {
        motor2.stopRotation();
      }
    }
}

/* encoder ISRs */

#ifdef M1_ENCODER_PORT
void m1_encoder_interrupt(void)
{
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
  UART_PORT.print("m1 ");
  UART_PORT.println(motor1.encoderCount);
  #endif

}

#endif
#ifdef M2_ENCODER_PORT
void m2_encoder_interrupt(void)
{
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += encoderStates[(oldEncoderState & 0x0F)];

  #ifdef DEBUG_ENCODERS
  UART_PORT.print("m2 ");
  // UART_PORT.println(oldEncoderState,BIN);
  // UART_PORT.println(" ");
  UART_PORT.println(motor2.encoderCount);
  #endif

}

#endif

/* limit switch ISRs */
void m1CwISR(void)
{
  motor1.stopRotation();
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
}

void m1CcwISR(void)
{
  motor1.stopRotation();
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
}

void m2FlexISR(void)
{
  motor2.stopRotation();
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
}

void m2ExtendISR(void)
{
  motor2.stopRotation();
  // should also alert the user somehow
  // should also perform some checks or update an angle somehow
}