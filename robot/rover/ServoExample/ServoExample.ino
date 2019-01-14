

#include "PinSetup.h"
#include "Parser.h"
#include "PidController.h"
#include "RobotMotor.h"
#include "ServoMotor.h"

// motor control interrupts main loop, encoders interrupt motor control, limit switches interrupt encoder calculations
#define LIMIT_SWITCH_NVIC_PRIORITY 100
#define ENCODER_NVIC_PRIORITY LIMIT_SWITCH_NVIC_PRIORITY + 4
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4
/* serial */
#define UART_PORT Serial
#define BAUD_RATE 9600 // serial bit rate
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

ServoMotor gripperMotor(PWM_PIN, GEAR_RATIO);
// motor array prep work: making pointers to motor objects
ServoMotor * m1 = & gripperMotor;
// I can use this instead of switch/case statements by doing motorArray[motornumber]->attribute
RobotMotor * motorArray[] =
{
  m1
};

IntervalTimer servoTimer; // motors 5&6
// these are a nicer way of timing events than using millis()
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent
/* function declarations */
void printMotorAngles(); // sends all motor angles over serial

void setup()
{
	pinSetup(); // initializes all the appropriate pins to outputs or interrupt pins etc
  UART_PORT.begin(BAUD_RATE);
  UART_PORT.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms
  gripperMotor.pidController.setJointAngleTolerance(2.0 * gripperMotor.gearRatioReciprocal); // randomly chosen for servo
  gripperMotor.setAngleLimits(MINIMUM_ANGLE, MAXIMUM_ANGLE);
  gripperMotor.pidController.setOutputLimits(-50, 50, 5.0);
  gripperMotor.isOpenLoop = true;
  gripperMotor.hasRamping = false;
  gripperMotor.openLoopSpeed = 50; // 50% speed
  gripperMotor.openLoopGain = 1.0; // totally random guess, needs to be tested
servoTimer.begin(servoInterrupt, SERVO_PID_PERIOD); // need to choose a period... went with 20ms because that's typical pwm period for servos...
  servoTimer.priority(MOTOR_NVIC_PRIORITY);
  // reset the elapsedMillis variables so that they're fresh upon entering the loop()
  sinceAnglePrint = 0;
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
    Parser.parseCommand(motorCommand, serialBuffer);
    memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
    if (!Parser.verifCommand(motorCommand))
    {
      // do nothing in particular
      //UART_PORT.println("$E,Error: verification failed");
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
                if (gripperMotor.setDesiredAngle(motorCommand.anglesToReach[0]))
                {
                  // this method returns true if the command is within joint angle limits
                  if (gripperMotor.isOpenLoop)
                  {
                    gripperMotor.openLoopError = gripperMotor.getDesiredAngle(); // - motor1.calcCurrentAngle(); // find the angle difference
                    gripperMotor.calcDirection(gripperMotor.openLoopError); // determine rotation direction and save the value
                    // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
                    if (gripperMotor.calcTurningDuration(gripperMotor.openLoopError))
                    // returns false if the open loop error is too small
                    {
                      gripperMotor.timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
                      gripperMotor.movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
                      UART_PORT.print("$S,Success: motor ");
                      UART_PORT.print(1);
                      UART_PORT.print(" to turn for ");
                      UART_PORT.print(gripperMotor.numMillis);
                      UART_PORT.println(" milliseconds");
                    }
                    else
                    {
                      UART_PORT.println("$E,Alert: requested angle is too close to current angle. Motor not changing course.");
                    }
                  }
                  else
                    if (!gripperMotor.isOpenLoop)
                    {
                      // all the heavy lifting for closed loop control is done in the timer interrupt
                      gripperMotor.movementDone = false;
                    }
                }
              }
            }
          }
        // reset the motor angle's variable or actually control the motor to reset it to neutral position
        else
          if (motorCommand.resetCommand)
          {
            if (motorCommand.resetAngleValue)
            {
              motorArray[motorCommand.whichMotor - 1] -> setCurrentAngle(0.0);
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
    if (motorArray[i] -> calcCurrentAngle())
    {
      UART_PORT.print(motorArray[i] -> getCurrentAngle());
      UART_PORT.print(", ");
    }
  }
  UART_PORT.println("");
}

void servoInterrupt(void)
{
  // movementDone can be set elsewhere... so are numMillis,
  // openLoopSpeed and rotationDirection (in open loop control)numMillis
  if (gripperMotor.isOpenLoop)
  {
    // open loop control
    if (!gripperMotor.movementDone && gripperMotor.timeCount < gripperMotor.numMillis)
    {
      gripperMotor.setVelocity(gripperMotor.rotationDirection, gripperMotor.openLoopSpeed);
    }
    else
    {
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      gripperMotor.movementDone = true;
      gripperMotor.stopRotation();
    }
  }
}
