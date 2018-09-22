#include "PinSetup.h"

#include "RobotMotor.h"
#include "StepperMotor.h"
#include "DCMotor.h"
#include "ServoMotor.h"

#define DC_PID_PERIOD 40000 // 40ms, because typical pwm signals have 20ms periods
#define SERVO_PID_PERIOD 40000 // 40ms, because typical pwm signals have 20ms periods
#define STEPPER_CHECK_INTERVAL 250 // every 250ms check if the stepper is in the right spot

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
  int whichDir = 0;
  int whichSpeed = 0;
  unsigned int whichTime = 0;
  bool angleCommand = false;
  float whichAngle = 0.0;
} budgeCommand, emptyBudgeCommand; // emptyBudgeCommand is used to reset the struct when the loop restarts

const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; //quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states

// instantiate motor objects here. only dcmotor currently supports interrupts
StepperMotor motor1(M1_ENABLE_PIN, M1_DIR_PIN, M1_STEP_PIN, M1_STEP_RESOLUTION, FULL_STEP, M1_GEAR_RATIO);
//DCMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B); // sabertooth
DCMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_GEAR_RATIO); // for new driver
StepperMotor motor3(M3_ENABLE_PIN, M3_DIR_PIN, M3_STEP_PIN, M3_STEP_RESOLUTION, FULL_STEP, M3_GEAR_RATIO);
StepperMotor motor4(M4_ENABLE_PIN, M4_DIR_PIN, M4_STEP_PIN, M4_STEP_RESOLUTION, FULL_STEP, M4_GEAR_RATIO);
ServoMotor motor5(M5_PWM_PIN, M5_GEAR_RATIO);
ServoMotor motor6(M6_PWM_PIN, M6_GEAR_RATIO);

IntervalTimer stepperTimer;
IntervalTimer dcTimer;
IntervalTimer servoTimer;

elapsedMillis sinceStepperCheck;

//RobotMotor motorArray[] = {motor1, motor2, motor3, motor4, motor5, motor6}; doesn't work rip
//StepperMotor stepperArray[] = {motor1, motor3, motor4};

void printMotorAngles();

void m1_encoder_interrupt(void);
void m2_encoder_interrupt(void);
void m3_encoder_interrupt(void);
void m4_encoder_interrupt(void);
//void m5_encoder_interrupt(void);
//void m6_encoder_interrupt(void);

void stepperInterrupt(void);
void dcInterrupt(void);
void servoInterrupt(void);

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

  //stepperTimer.begin(stepperInterrupt, STEP_INTERVAL1 * 1000); //25ms
  //dcTimer.begin(dcInterrupt, DC_PID_PERIOD); //need to choose a period... went with 20ms because that's typical pwm period for servos...
  //servoTimer.begin(dcInterrupt, SERVO_PID_PERIOD); //need to choose a period... went with 20ms because that's typical pwm period for servos...

  sinceAnglePrint = 0;
  sinceStepperCheck = 0;
}

void loop() {

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.println("=======================================================");
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received
    char* msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // look for first element (first tag)
    if (String(msgElem) == "motor") { // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (motor number)
      int tempMotorVar = atoi(msgElem);
      // currently uses motor1's numMotors variable which is shared by all RoverMotor objects and children. need better implementation
      if (tempMotorVar > 0 && tempMotorVar <= RobotMotor::numMotors) {
        budgeCommand.whichMotor = tempMotorVar;
        Serial.print("parsed motor "); Serial.println(budgeCommand.whichMotor);
      }
      else Serial.println("motor does not exist");
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (direction tag)
      if (String(msgElem) == "angle") { // msgElem is a char array so it's safer to convert to string first
        budgeCommand.angleCommand = true;
        msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (desired angle value)
        float tempAngleVar = atof(msgElem); // converts to float
        if (tempAngleVar > -720.0 && tempAngleVar < 720.0) {
          budgeCommand.whichAngle = tempAngleVar;
          Serial.print("parsed desired angle "); Serial.println(budgeCommand.whichAngle);
        }
        else Serial.println("angle is out of bounds");
      }
      else if (String(msgElem) == "direction") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (direction value)
        switch (*msgElem) { // determines motor direction
          case '0': // arbitrarily (for now) decided 0 is clockwise
            budgeCommand.whichDir = CLOCKWISE;
            Serial.println("parsed direction clockwise");
            break;
          case '1': // arbitrarily (for now) decided 1 is counter-clockwise
            budgeCommand.whichDir = COUNTER_CLOCKWISE;
            Serial.println("parsed direction counter-clockwise");
            break;
        }
        msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (speed tag)
        if (String(msgElem) == "speed") { // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (integer representing speed level)
          int tempSpeedVar = atoi(msgElem); // converts to int
          if (tempSpeedVar <= MAX_SPEED) { // make sure the subtraction made sense. Tf it's above 9, it doesn't
            budgeCommand.whichSpeed = tempSpeedVar + 1; // set the actual speed, enum starts with 1
            Serial.print("parsed speed level: "); Serial.println(budgeCommand.whichSpeed - 1); // minus 1 to be consistent with incoming message
          }
        }
        msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (time tag)
        if (String(msgElem) == "time") { // msgElem is a char array so it's safer to convert to string first
          msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (time in seconds)
          unsigned int tempTimeVar = atoi(msgElem); // converts to int
          if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
            budgeCommand.whichTime = tempTimeVar;
            Serial.print("parsed time interval "); Serial.print(budgeCommand.whichTime); Serial.println("ms");
          }
        }
      }
    }
    memset(serialBuffer, 0, BUFFER_SIZE); //empty the buffer
    restOfMessage = serialBuffer; // reset pointer
  }

  if (budgeCommand.whichMotor > 0) {
    if (budgeCommand.angleCommand) {
      Serial.print("motor "); Serial.print(budgeCommand.whichMotor); Serial.print(" desired angle (degrees) is: "); Serial.println(budgeCommand.whichAngle);
      Serial.println("=======================================================");
      switch (budgeCommand.whichMotor) { // move a motor based on which one was commanded
        case MOTOR1:
          //motor1.movementDone = false;
          //motor1.enablePower();
          // If I do the code like this, it means that once the motor achieves the correct position,
          // it will stop and never recorrect until a new angle request is sent.
          // Also, technically updatePID should only be called in the stepper interrupt.
          // Well, technically it should only be used for the DC motor....
          motor1.desiredAngle = budgeCommand.whichAngle;
          motor1.motorPID.updatePID(motor1.currentAngle, motor1.desiredAngle);
          break;
        case MOTOR2:
          motor2.desiredAngle = budgeCommand.whichAngle;
          motor2.motorPID.updatePID(motor2.currentAngle, motor2.desiredAngle);
          break;
        case MOTOR3:
          {
            motor3.desiredAngle = budgeCommand.whichAngle; // set the desired angle based on the command
            motor3.motorPID.openLoopError = motor3.desiredAngle - motor3.getCurrentAngle(); // find the angle difference

            // determine the direction
            if (motor3.motorPID.openLoopError >= 0) motor3.motorPID.openLoopDir = 1;
            else motor3.motorPID.openLoopDir = -1;

            // if the error is big enough to justify movement
            if (fabs(motor3.motorPID.openLoopError) > motor3.motorPID.angleTolerance) {
              motor3.motorPID.numSteps = fabs(motor3.motorPID.openLoopError) / motor3.stepResolution; // calculate the number of steps to take
              motor3.enablePower(); // give power to the stepper finally
              motor3.motorPID.movementDone = false; // this flag being false lets the timer interrupt move the stepper
            }
            //motor3.motorPID.updatePID(motor3.currentAngle, motor3.desiredAngle);
            break;
          }
        case MOTOR4:
          //motor4.movementDone = false;
          //motor4.enablePower();
          motor4.desiredAngle = budgeCommand.whichAngle;
          motor4.motorPID.updatePID(motor4.currentAngle, motor4.desiredAngle);
          break;
        case MOTOR5:
          //motor5.motorPID.updatePID(motor5.currentAngle, motor5.desiredAngle);
          break;
        case MOTOR6:
          //motor6.motorPID.updatePID(motor6.currentAngle, motor6.desiredAngle);
          break;
      }
    }
    else if (budgeCommand.whichDir > 0 && budgeCommand.whichSpeed > 0 && budgeCommand.whichTime > 0) {
      Serial.print("motor "); Serial.print(budgeCommand.whichMotor); Serial.println(" to move");
      Serial.println("=======================================================");

      //motorArray[budgeCommand.whichMotor-1].budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
      switch (budgeCommand.whichMotor) { // move a motor based on which one was commanded
        case MOTOR1:
          motor1.budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
          break;
        case MOTOR2:
          motor2.budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
          break;
        case MOTOR3:
          motor3.budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
          break;
        case MOTOR4:
          motor4.budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
          break;
        case MOTOR5:
          motor5.budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
          break;
        case MOTOR6:
          motor6.budge(budgeCommand.whichDir, budgeCommand.whichSpeed, budgeCommand.whichTime);
          break;
      }
    }
    else Serial.println("$E,Error: bad motor command");
  }
  budgeCommand = emptyBudgeCommand; // reset budgeCommand so the microcontroller doesn't try to move a motor next loop

  if (sinceStepperCheck >= STEPPER_CHECK_INTERVAL) {
    // all of this code should probably go into a function called "calculate motor steps" or something...
    // this code is very similar to what happens above when it decides which motor should turn after receiving a command
    // this code also assumes that the correct amount of steps will take it to the right spot
    // this means that it doesn't account for faulty angle calculations from the encoder or the motor resolution...
    int remainingSteps = motor3.motorPID.numSteps - motor3.motorPID.stepCount;
    float imaginedRemainingAngle = remainingSteps * motor3.stepResolution; // how far does the motor think it needs to go
    float actualRemainingAngle = motor3.desiredAngle - motor3.getCurrentAngle(); // how far does it actually need to go
    float discrepancy = actualRemainingAngle - imaginedRemainingAngle ;

    if (discrepancy >= 0) motor3.motorPID.openLoopDir = 1;
    else discrepancy = -1;

    if (fabs(discrepancy) > motor3.motorPID.angleTolerance) {
      motor3.motorPID.numSteps = fabs(discrepancy) / motor3.stepResolution; // calculate the number of steps to take
      motor3.enablePower(); // give power to the stepper finally
      motor3.motorPID.movementDone = false; // this flag being false lets the timer interrupt move the stepper
    }
    sinceStepperCheck = 0;
  }

  if (motor1.motorPID.movementDone) motor1.disablePower();
  if (motor3.motorPID.movementDone) motor3.disablePower();
  if (motor4.motorPID.movementDone) motor4.disablePower();

  // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) {
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }

}

void printMotorAngles() {
  Serial.print("Motor Angles: ");
  Serial.print(motor1.getCurrentAngle()); Serial.print(",");
  Serial.print(motor2.getCurrentAngle()); Serial.print(",");
  Serial.print(motor3.getCurrentAngle()); Serial.print(",");
  Serial.println(motor4.getCurrentAngle());//Serial.print(",");
  //Serial.print(motor5.getCurrentAngle());Serial.print(",");
  //Serial.print(motor6.getCurrentAngle());Serial.print(",");
}

/*
  void stepperInterrupt(void) {
  static int nextInterval = 0;
  int i = 3;
  // code to decide which motor to turn goes here, or code just turns all motors
  if (!stepperArray[i - 1].movementDone) {
    // code to decide how fast the motor will turn and in which direction
    int dir = CLOCKWISE;
    nextInterval = 25000;
    //set a minimum step period somewhere
    //nextInterval = stepperArray[i-1].motorPID.updatePID(stepperArray[i-1].currentAngle, stepperArray[i-1].desiredAngle);
    stepperArray[i - 1].singleStep(dir);
    stepperTimer.update(nextInterval);
  }
  }
*/

// this function has constant step interval
void stepperInterrupt(void) {
  static int nextInterval = 0;
  if (!motor3.motorPID.movementDone) {
    motor3.singleStep(motor3.motorPID.openLoopDir); // direction was set beforehand
    motor3.motorPID.stepCount++;
    nextInterval = stepIntervalArray[1] * 1000; // speed 2 is 25ms steps
    stepperTimer.update(nextInterval); // need to check if im allowed to call this within the interrupt
  }
  else motor3.motorPID.stepCount = 0; // reset the counter
}

void dcInterrupt(void) {
  // code to decide which motor to turn goes here, or code just turns all motors
  if (!motor2.motorPID.movementDone)
    motor2.getCurrentAngle();
  // rethink the PID? needs to set a direction AND a speed
  motor2.motorPID.updatePID(motor2.currentAngle, motor2.desiredAngle);
  int motorSpeed = motor2.motorPID.pidOutput * 255; // 255 is arbitrary value... some conversion probably required between pid output and motor speed input
  int dir = CLOCKWISE;
  motor2.setVelocity(dir, motorSpeed);
}

void servoInterrupt(void) {
  ;
}

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
  motor1.encoderCount += dir[(oldEncoderState & 0x0F)]; // clear the higher bits. The dir[] array corresponds to the correct direction for a specific set of prev and current encoder states
}
void m2_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m2 "); Serial.println(motor2.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M2_ENCODER_PORT >> M2_ENCODER_SHIFT) & 0x03);
  motor2.encoderCount += dir[(oldEncoderState & 0x0F)];
}
void m3_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.println("m3 "); //Serial.println(motor3.encoderCount);
  //Serial.println(M3_ENCODER_PORT,BIN);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M3_ENCODER_PORT >> M3_ENCODER_SHIFT) & 0x03);
  motor3.encoderCount += dir[(oldEncoderState & 0x0F)];
}
void m4_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m4 "); Serial.println(motor4.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M4_ENCODER_PORT >> M4_ENCODER_SHIFT) & 0x03);
  motor4.encoderCount += dir[(oldEncoderState & 0x0F)];
}
/*
  void m5_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m5 "); Serial.println(motor5.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M5_ENCODER_PORT >> M5_ENCODER_SHIFT) & 0x03);
  motor5.encoderCount += dir[(oldEncoderState & 0x0F)];
  }
  void m6_encoder_interrupt(void) {
  static unsigned int oldEncoderState = 0;
  Serial.print("m6 "); Serial.println(motor6.encoderCount);
  oldEncoderState <<= 2;
  oldEncoderState |= ((M6_ENCODER_PORT >> M6_ENCODER_SHIFT) & 0x03);
  motor6.encoderCount += dir[(oldEncoderState & 0x0F)];
  }
*/
