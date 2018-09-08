/*
  TODO:
  -suggesgtion 1 pinmode input, 2 is micros pwm instead of analogwrite (timer is bad?)
  -(always) implement error checking, remove parts later on if it's too slow (tatum doubts)
  -(always) clean up code, comment code
  -(later) implement power management? sleep mode?
  -(later) implement PID for automatic control
    -update parsing function to take in angles or just go directly to ROSserial, probably the former
  -(done) solve many problems by creating motor type subclasses
  -(done) leave serial.print debugging messages unless it really takes too much time (doubt)
  -(done) figure out how to solve the servo jitter. setting pins to INPUT probably fixes it
  -(depends on wiring) determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
  -(need abtin's magic) figure out how to step up to 5v for the sabertooth
  -(done-ish) figure out how to send motor info from parsing function to budgeMotor etc
  -(done) instead of resetting budgeCommand, perform some kind of check so that the motor doesn't spin forever?

  -(done) implement budge function:
   -(done) parsing procedure can process direction, speed and time, which have defaults in budge()
   -(done) implement limit for max turns in right/left directions
   -(done) connect and test all motor types with budge()

  -(now) determine motor angles thru encoder interrupts
   -(done) replace strtok with strtok_r when implementing interrupts
   -(done) rewrite all the register bit variables to use teensy registers for encoder interrupts
   -(now) figure out better way to check max motor count than accessing motor1 object
   -(now) solve encoder direction change issue? not necessary
   -(now) ensure interrupts function with new class structure - can't pass GPIOx_PDIr through variable?
   -(next) confirm all the pins will work with interrupts and not stepping on each other
   -figure out encoder resolutions and gear ratios
   -(next) make sure there is  max/min angle limitation with encoders
   -determine whether it's worth it to use the built in quadrature decoders

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
   -(next) decide whether to use abtin's software interrupt or simpler method for simultaneous motor control
    -implement appropriate software interrupts that can take input from PID or manual control functions?
    -decide frequency of motor control loop: figure out estimated time for loop
    -rewrite the motor control with timers for teensy, ensure control for all motors

  -(next next step) external interrupts for limit switches
   -rewrite all the register bit variables to use teensy registers for limit switch interrupts
   -incorporate limit switches for homing position on all motors
   -implement homing function on boot
*/

#include "PinSetup.h"

#include "RoverMotor.h"
#include "StepperMotor.h"
#include "DCMotor.h"
#include "ServoMotor.h"

/* serial */
#define BAUD_RATE 115200 // serial baud rate

/* parsing */
#define BUFFER_SIZE 100  // size of the buffer for the serial commands

char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
char *restOfMessage = serialBuffer; // used in strtok_r, which is the reentrant version of strtok
int tempMotorVar; // checks the motor before giving the data to the struct below
int tempSpeedVar; // checks the speed before giving the data to the struct below
unsigned int tempTimeVar; // checks the time before giving the data to the struct below

struct budgeInfo { // info from parsing functionality is packaged and given to motor control functionality
  int whichMotor = 0;
  int whichDir = 0;
  int whichSpeed = 0;
  unsigned int whichTime = 0;
} budgeCommand, emptyBudgeCommand; // emptyBudgeCommand is used to reset the struct when the loop restarts

/* motor contruction */
// to reduce constructor parameters i wanted to make structs but it may make the code even messier.. try typedefs?
/*
  struct dcPins {

  };

  struct servoPins{

  };

  struct stepperPins {

  };
*/

const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; //quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states

// instantiate motor objects here. only dcmotor currently supports interrupts
StepperMotor motor1(M1_ENABLE_PIN, M1_DIR_PIN, M1_STEP_PIN, M1_ENCODER_A, M1_ENCODER_B, M1_ENCODER_PORT, M1_ENCODER_SHIFT);
//DCMotor motor2(M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B); // sabertooth
DCMotor motor2(M2_DIR_PIN, M2_PWM_PIN, M2_ENCODER_A, M2_ENCODER_B, M2_ENCODER_PORT, M2_ENCODER_SHIFT); // for new driver
StepperMotor motor3(M3_ENABLE_PIN, M3_DIR_PIN, M3_STEP_PIN, M3_ENCODER_A, M3_ENCODER_B, M3_ENCODER_PORT, M3_ENCODER_SHIFT);
StepperMotor motor4(M4_ENABLE_PIN, M4_DIR_PIN, M4_STEP_PIN, M4_ENCODER_A, M4_ENCODER_B, M4_ENCODER_PORT, M4_ENCODER_SHIFT);
ServoMotor motor5(M5_PWM_PIN);
ServoMotor motor6(M6_PWM_PIN);

/*
  // create wrappers to circumvent C++ refusing to attach instance-dependent interrupts inside a class
  void m1WrapperISR(void) {
  motor1.encoder_interrupt();
  }
  void m2WrapperISR(void) {
  motor2.encoder_interrupt();
  }
  void m3WrapperISR(void) {
  motor3.encoder_interrupt();
  }
  void m4WrapperISR(void) {
  motor4.encoder_interrupt();
  }
  //void m5WrapperISR(void){ motor5.encoder_interrupt(); }
  //void m6WrapperISR(void){ motor6.encoder_interrupt(); }
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

void setup() {
  pinSetup();
  Serial.begin(BAUD_RATE); Serial.setTimeout(50); // checks serial port every 50ms

  // to clean up: each motor needs to attach 2 interrupts, which is a lot of lines of code
  attachInterrupt(motor1.encoderPinA, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor1.encoderPinB, m1_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinA, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor2.encoderPinB, m2_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinA, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor3.encoderPinB, m3_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinA, m4_encoder_interrupt, CHANGE);
  attachInterrupt(motor4.encoderPinB, m4_encoder_interrupt, CHANGE);

  /*
    attachInterrupt(motor1.encoderPinA, m1WrapperISR, CHANGE);
    attachInterrupt(motor1.encoderPinB, m1WrapperISR, CHANGE);

    attachInterrupt(motor2.encoderPinA, m2WrapperISR, CHANGE);
    attachInterrupt(motor2.encoderPinB, m2WrapperISR, CHANGE);

    attachInterrupt(motor3.encoderPinA, m3WrapperISR, CHANGE);
    attachInterrupt(motor3.encoderPinB, m3WrapperISR, CHANGE);

    attachInterrupt(motor4.encoderPinA, m4WrapperISR, CHANGE);
    attachInterrupt(motor4.encoderPinB, m4WrapperISR, CHANGE);
  */
  /*
    attachInterrupt(motor5.encoderPinA, m5WrapperISR, CHANGE);
    attachInterrupt(motor5.encoderPinB, m5WrapperISR, CHANGE);

    attachInterrupt(motor6.encoderPinA, m6WrapperISR, CHANGE);
    attachInterrupt(motor6.encoderPinB, m6WrapperISR, CHANGE);
  */
}

void loop() {

  if (Serial.available()) { // if a message was sent to the Teensy
    Serial.println("=======================================================");
    Serial.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
    Serial.print("GOT: "); Serial.println(serialBuffer); // send back what was received
    char* msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // look for first element (first tag)
    if (String(msgElem) == "motor") { // msgElem is a char array so it's safer to convert to string first
      msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (motor number)
      tempMotorVar = atoi(msgElem);
      // currently uses motor1's numMotors variable which is shared by all RoverMotor objects and children. need better implementation
      if (tempMotorVar > 0 && tempMotorVar <= motor1.numMotors) {
        budgeCommand.whichMotor = tempMotorVar;
        Serial.print("parsed motor "); Serial.println(budgeCommand.whichMotor);
      }
      else Serial.println("motor does not exist");
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (direction tag)
      if (String(msgElem) == "direction") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // go to next msg element (direction)
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
      }
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (speed tag)
      if (String(msgElem) == "speed") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (integer representing speed level)
        tempSpeedVar = atoi(msgElem); // converts to int
        if (tempSpeedVar <= MAX_SPEED) { // make sure the subtraction made sense. Tf it's above 9, it doesn't
          budgeCommand.whichSpeed = tempSpeedVar + 1; // set the actual speed, enum starts with 1
          Serial.print("parsed speed level: "); Serial.println(budgeCommand.whichSpeed - 1); // minus 1 to be consistent with incoming message
        }
      }
      msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (time tag)
      if (String(msgElem) == "time") { // msgElem is a char array so it's safer to convert to string first
        msgElem = strtok_r(NULL, " ", &restOfMessage); // find the next message element (time in seconds)
        tempTimeVar = atoi(msgElem); // converts to int
        if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
          budgeCommand.whichTime = tempTimeVar;
          Serial.print("parsed time interval "); Serial.print(budgeCommand.whichTime); Serial.println("ms");
        }
      }
    }
    memset(serialBuffer, 0, BUFFER_SIZE); //empty the buffer
    restOfMessage = serialBuffer; // reset pointer
  }

  if (budgeCommand.whichMotor > 0) {
    if (budgeCommand.whichDir > 0 && budgeCommand.whichSpeed > 0 && budgeCommand.whichTime > 0) {
      Serial.print("motor "); Serial.print(budgeCommand.whichMotor); Serial.println(" to move");
      Serial.println("=======================================================");
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
    else Serial.println("bad motor command");
  }
  budgeCommand = emptyBudgeCommand; // reset budgeCommand so the microcontroller doesn't try to move a motor next loop

}
