#define DEBUG_MAIN 10 // debug messages during main loop
//#define DEBUG_PARSING 11 // debug messages during parsing function
//#define DEBUG_VERIFYING 12 // debug messages during verification function
//#define DEBUG_ENCODERS 13 // debug messages during encoder interrupts
//#define DEBUG_PID 14 // debug messages during pid loop calculations
//#define DEBUG_DC_TIMER 16 // debug messages during dc timer interrupts
//#define DEBUG_SERVO_TIMER 17 // debug messages during servo timer interrupts
//#define DEBUG_STEPPER_3_TIMER 18 // debug messages during stepper 3 timer interrupts
//#define DEBUG_STEPPER_4_TIMER 19 // debug messages during stepper 3 timer interrupts

#define UART_PORT Serial

#include <Servo.h>
#include "PinSetup.h"
#include "Parser.h"
#include "PidController.h"
#include "RobotMotor.h"
#include "StepperMotor.h"
#include "DcMotor.h"
#include "ServoMotor.h"

/* interrupt priorities */
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
commandInfo motorCommand, emptyMotorCommand; // emptyMotorCommand is used to reset the struct when the loop restarts
Parser Parser; // object which parses and verifies commands
bool msgReceived = false; bool msgIsValid = false;

// instantiate motor objects here:
DcMotor motor1(M1_DIR_PIN, M1_PWM_PIN, M1_GEAR_RATIO);
DcMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO);
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
RobotMotor * motorArray[] = { m1, m2, m3, m4, m5, m6 };

// instantiate timers here:
IntervalTimer dcTimer; // motors 1&2
IntervalTimer m3StepperTimer;
IntervalTimer m4StepperTimer;
IntervalTimer servoTimer; // motors 5&6
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent

void printMotorAngles(void); // sends all motor angles over serial
void dcInterrupt(void); // manages motors 1&2
void servoInterrupt(void); // manages motors 5&6
void m3StepperInterrupt(void);
void m4StepperInterrupt(void);

/* Teensy setup */
void setup() {
  pinSetup(); // initializes all the appropriate pins to outputs or interrupt pins etc
  UART_PORT.begin(BAUD_RATE);
  UART_PORT.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms

  // set motor shaft angle tolerances
  motor1.pidController.setJointAngleTolerance(2.0 * motor1.gearRatioReciprocal); // randomly chosen for dc
  motor2.pidController.setJointAngleTolerance(2.0 * motor2.gearRatioReciprocal);
  motor3.pidController.setJointAngleTolerance(1.8 * 2 * motor3.gearRatioReciprocal); // 1.8 is the min stepper resolution so I gave it +/- tolerance
  motor4.pidController.setJointAngleTolerance(1.8 * 2 * motor4.gearRatioReciprocal);
  motor5.pidController.setJointAngleTolerance(2.0 * motor5.gearRatioReciprocal); // randomly chosen for servo
  motor6.pidController.setJointAngleTolerance(2.0 * motor6.gearRatioReciprocal);
  // set motor joint angle limits
  motor1.setAngleLimits(M1_MINIMUM_ANGLE, M1_MAXIMUM_ANGLE);
  motor2.setAngleLimits(M2_MINIMUM_ANGLE, M2_MAXIMUM_ANGLE);
  motor3.setAngleLimits(M3_MINIMUM_ANGLE, M3_MAXIMUM_ANGLE);
  motor4.setAngleLimits(M4_MINIMUM_ANGLE, M4_MAXIMUM_ANGLE);
  // motor5.setAngleLimits(M5_MINIMUM_ANGLE, M5_MAXIMUM_ANGLE); // this joint should be able to spin freely
  motor6.setAngleLimits(M6_MINIMUM_ANGLE, M6_MAXIMUM_ANGLE);
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
  motor4.isOpenLoop = true;
  motor4.hasRamping = false;
  motor4.openLoopSpeed = 50; // 50% speed
  motor5.isOpenLoop = true;
  motor5.hasRamping = false;
  motor5.openLoopSpeed = 50; // 50% speed
  motor5.openLoopGain = 0.32; // for 5V
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
}

/* main code loop */
void loop() {
  motorCommand = emptyMotorCommand; // reset motorCommand so the microcontroller doesn't try to move a motor next loop
  msgReceived = false;  msgIsValid = false;
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
  if (msgReceived) {
    if (msgIsValid) {
#ifdef DEBUG_MAIN
      UART_PORT.println(messageBar);
#endif
      if (motorCommand.pingCommand) {
        // respond to ping
        UART_PORT.println("pong");
      }
      else if (motorCommand.stopAllMotors) { // emergency stop takes precedence
        for (int i = 0; i < NUM_MOTORS; i++) {
          motorArray[i] -> stopRotation();
        }
        UART_PORT.println("all motors stopped because of emergency stop");
      }
      else { // following cases are for commands to specific motors
        if (motorCommand.budgeCommand) { // make motors move until the command isn't sent anymore
          for (int i = 0; i < NUM_MOTORS; i++) {
            if (motorCommand.motorsToMove[i]) {
              UART_PORT.print("motor ");
              UART_PORT.print(i + 1);
              UART_PORT.print(" desired direction is: ");
              UART_PORT.println(motorCommand.directionsToMove[i]);
            }
          }
        }
        bool motorsCanMove = true;
        for (int i = 0; i < NUM_MOTORS; i++) {
          if (motorCommand.budgeCommand) {
            motorArray[i] -> calcCurrentAngle();
            int dir = motorCommand.directionsToMove[i];
            float ang = motorArray[i] -> getImaginedAngle();
            if (motorArray[i] -> hasAngleLimits) {
              if ( ( (dir > 0) && (ang > motorArray[i] -> maxJointAngle) ) || ( (dir < 0) && (ang < motorArray[i] -> minJointAngle) ) ) {
                motorsCanMove = false;
              }
            }
          }
        }
        if (!motorsCanMove) {
          UART_PORT.println("$E,Error: one or many angles are invalid, arm will not move");
        }
        else { // start moving things
          if (motorCommand.motorsToMove[0]) { // this motor can swivel but has limits so it doesn't hit anything
            if (motorCommand.budgeCommand) {
              motor1.calcDirection(motorCommand.directionsToMove[0]); // yes, this works
              motor1.isBudging = true;
              motor1.movementDone = false;
              motor1.sinceBudgeCommand = 0;
              motor1.startAngle = motor1.getImaginedAngle();
            }
          }
          if (motorCommand.motorsToMove[1]) {
            if (motorCommand.budgeCommand) {
              motor2.calcDirection(motorCommand.directionsToMove[1]); // yes, this works
              motor2.isBudging = true;
              motor2.movementDone = false;
              motor2.sinceBudgeCommand = 0;
              motor2.startAngle = motor2.getImaginedAngle();
            }
          }
          if (motorCommand.motorsToMove[2]) {
            if (motorCommand.budgeCommand) {
              motor3.calcDirection(motorCommand.directionsToMove[2]); // yes, this works
              motor3.isBudging = true;
              motor3.movementDone = false;
              motor3.stepCount = 0;
              motor3.sinceBudgeCommand = 0;
              motor3.startAngle = motor3.getImaginedAngle();
              Serial.print("m3 start angle ");
              Serial.println(motor3.startAngle);
            }
          }
          if (motorCommand.motorsToMove[3]) {
            if (motorCommand.budgeCommand) {
              motor4.calcDirection(motorCommand.directionsToMove[3]); // yes, this works
              motor4.isBudging = true;
              motor4.movementDone = false;
              motor4.stepCount = 0;
              motor4.sinceBudgeCommand = 0;
              motor4.startAngle = motor4.getImaginedAngle();
            }
          }
          if (motorCommand.motorsToMove[4]) {
            // this motor has no max or min angle because it must be able to spin like a screwdriver
            if (motorCommand.budgeCommand) {
              motor5.calcDirection(motorCommand.directionsToMove[4]); // yes, this works
              motor5.isBudging = true;
              motor5.movementDone = false;
              motor5.sinceBudgeCommand = 0;
              motor5.startAngle = motor5.getImaginedAngle();
            }
          }
          if (motorCommand.motorsToMove[5]) {
            if (motorCommand.budgeCommand) {
              motor6.calcDirection(motorCommand.directionsToMove[5]); // yes, this works
              motor6.isBudging = true;
              motor6.movementDone = false;
              motor6.sinceBudgeCommand = 0;
              motor6.startAngle = motor6.getImaginedAngle();
            }
          }
        }
      }
    }
    else { // bad command
      UART_PORT.println("$E,Error: bad motor command");
    }
  }
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) { // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }
}

void printMotorAngles(void) {
  UART_PORT.print("Motor Angles: ");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorArray[i] -> isOpenLoop) {
      UART_PORT.print(motorArray[i] -> getImaginedAngle());
    }
    UART_PORT.print(", ");
  }
  UART_PORT.println("");
}

void m3StepperInterrupt(void) {
  motor3.nextInterval = STEPPER_PID_PERIOD; // how long until the next step is taken? indirectly controls speed
  if (motor3.isBudging) {
    if (motor3.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor3.calcCurrentAngle();
      motor3.setVelocity(motor3.rotationDirection, motor3.openLoopSpeed);
      motor3.stepCount++;
    }
    else {
      motor3.isBudging = false;
      motor3.movementDone = true;
      motor3.stopRotation();
    }
  }
}

void m4StepperInterrupt(void) {
  motor4.nextInterval = STEPPER_PID_PERIOD; // how long until the next step is taken? indirectly controls speed
  if (motor4.isBudging) {
    if (motor4.sinceBudgeCommand < BUDGE_TIMEOUT) {
      motor4.calcCurrentAngle();
      motor4.setVelocity(motor4.rotationDirection, motor4.openLoopSpeed);
      motor4.stepCount++;
    }
    else {
      motor4.isBudging = false;
      motor4.movementDone = true;
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
}

void servoInterrupt(void) {
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
}
