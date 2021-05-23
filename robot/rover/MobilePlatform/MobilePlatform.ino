//
// Edited by Michael on 2021-03-20.
//

// USB : Debug, UART : Production
#define USB

#include <cstdint>
#include "Navigation.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include "ArduinoBlue.h"

// Issue made to change includes to include
// #include "../internal_comms/include/CommandCenter.h"
#include "CommandCenter.h"
#include "../internal_comms/include/CommandCenter.h"
#include "includes/commands/WheelsCommandCenter.h"    
#include "includes/Commands.h"  // This automatically includes DcMotor.h
#include "includes/Globals.h"
#include "includes/PidController.h"
#include "includes/PinSetup.h"

/* Global variables*/
elapsedMillis sinceFeedbackPrint; // timer for sending motor speeds and battery measurements
elapsedMillis sinceLedToggle; // timer for heartbeat
elapsedMillis sinceSensorRead; // timer for reading battery, gps and imu data
elapsedMillis sinceMC; // timer for reading battery, gps and imu data
float linearVelocity, rotationalVelocity, rightLinearVelocity, leftLinearVelocity;
String rotation; // Rotation direction of the whole rover

// Specific values for calculations
const float wheelBase = 0.33; //distance between left and right wheels
const float radius = 0.14; // in m
const float piRad = 0.10472; // Pi in radians
const float kp = 14.1;
const float ki = 0.282;
const float kd = 40.625;

// Pins for Serial
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t ENABLE_PIN = 10; // TEMPORARY BEFORE FLOW CONTROL IMPLEMENTED

// Motor constructor initializations
DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO, "Front Right Motor");  // Motor 0
DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO, "Middle Right Motor"); // Motor 1
DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO, "Rear Right Motor"); // Motor 2
DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO, "Front Left Motor"); // Motor 3
DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO, "Middle Left Motor"); // Motor 4
DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO, "Rear Left Motor"); // Motor 5

// List of motors
DcMotor motorList[] = {RF, RM, RB, LF, LM, LB}; // 0,1,2,3,4,5 motors
Servo frontSide, frontBase, rearSide, rearBase;
Servo servoList[] = {frontSide, frontBase, rearSide, rearBase};

// Pointers to DC motors
DcMotor* RFPtr = &RF;
DcMotor* RMPtr = &RM;
DcMotor* RBPtr = &RB;
DcMotor* LFPtr = &LF;
DcMotor* LMPtr = &LM;
DcMotor* LBPtr = &LB;
DcMotor* motorPtrList[] = {RFPtr, RMPtr, RBPtr, LFPtr, LMPtr, LBPtr};

// Pointers to servo motors
Servo* frontSidePtr = &frontSide;
Servo* frontBasePtr = &frontBase;
Servo* rearSidePtr = &rearSide;
Servo* rearBasePtr = &rearBase;
Servo* servoPtrList[] = {frontSidePtr, frontBasePtr, rearSidePtr, rearBasePtr};

// Modified, used only to store values 
Commands Cmds;

// To read commands from wheelscommandcenter
internal_comms::CommandCenter* commandCenter = new WheelsCommandCenter();

/* Function declarations */
// initializers
void attachServos(void); // attach pins to servo objects
void roverVelocityCalculator(void);

// Initialize motor encoders
void initMotorEncoder0(void);
void initMotorEncoder1(void);
void initMotorEncoder2(void);
void initMotorEncoder3(void);
void initMotorEncoder4(void);
void initMotorEncoder5(void);
void initEncoders(void); // Encoder initiation (attach interrups and pinModes for wheel encoders

// Set encoder interrupts
void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);

// Initialization Serial
void initSerialCommunications(void);

// Wheel command methods from WheelsCommandCenter.cpp
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371
void setMotorList(DcMotor* motorPtrList);
void setServoList(Servo* servoPtrList);
DcMotor* getMotorList();
void toggleMotors(bool turnMotorOn);
void stopMotors(void);
void closeMotorsLoop(void);
void openMotorsLoop(void);
void toggleJoystick(bool turnJoystickOn);
void toggleGps(bool turnGpsOn);
void toggleEncoder(bool turnEncOn);
void toggleAcceleration(bool turnAccelOn);
void getRoverStatus(void);
void moveRover(int8_t roverThrottle, int8_t roverSteering); // Throttle -49 to 49 and Steering -49 to 49
void moveWheel(uint8_t wheelNumber, int16_t wheelPWM); // Wheel number 0 to 5 and -255 to 255

// Messages to send back to OBC from wheel Teensy
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371
void getLinearVelocity(void);
void getRotationalVelocity(void);
void getCurrentVelocity(void);
void getDesiredVelocity(void);
void getBatteryVoltage(void);
void pingWheels(void);

// Initial teensy setup
void setup() {
  initSerialCommunications();

  // Initialize setup for pins from PinSetup.h
  initPins();
  initEncoders();

  // Initialize servo motors
  attachServos();

  // Handle navigation commands each time a new command is received 
  // Looped and called as new commands are received 
  initNav(Cmds);

  // Use PID
  if (Cmds.isOpenLoop) {
    maxOutputSignal = MAX_PWM_VALUE;
    minOutputSignal = MIN_PWM_VALUE;
  }
  // Do not use PID
  else {
    maxOutputSignal = MAX_RPM_VALUE;
    minOutputSignal = MIN_RPM_VALUE;
  }
}

// Running the wheels
void loop() {
  // Acquire method based on command sent from serial
  if (Serial.available()) {
    /* 
    1. Pointer to select the method (WheelsCommandCenter.cpp) to run based on the command
    2. Read command performs executeCommand()
    3. If the serial is not enabled (such as being used by the arm), then the command skips this
    */
    commandCenter->readCommand();
  }

  if (sinceSensorRead > SENSOR_READ_INTERVAL) {
    getBatteryVoltage();
    navHandler(Cmds);
    sinceSensorRead = 0;
  }

  if (sinceLedToggle > LED_BLINK_INTERVAL) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    sinceLedToggle = 0;
  }

  if (Cmds.sinceThrottle > THROTTLE_TIMEOUT && Cmds.throttleTimeOut) {
    stopMotors();
  }

  if (sinceMC > MOTOR_CONTROL_INTERVAL && Cmds.isActivated) {
    // Loop through motors to get and set their velocity
    for (int i = 0; i < NUM_MOTORS; i++) {
      motorPtrList[i]->calcCurrentVelocity();
      motorPtrList[i]->setVelocity(motorPtrList[i]->desiredDirection, fabs(motorPtrList[i]->desiredVelocity), motorPtrList[i]->getCurrentVelocity());
    }
    sinceMC = 0;
  }

  /*
  Check if the message is available to be sent
  If available, send the message to be read and pop it out of the message queue
  If the message is unavailable, the message isn't removed from the queue
  */
  commandCenter->sendMessage();
}

void setMotorList(DcMotor* motorPtrList){
  Cmds.motorList = motorPtrList;
}

void setServoList(Servo* servoPtrList){
  Cmds.servoList = servoPtrList;
}

DcMotor* getMotorList() {
  return *motorPtrList;
}

void roverVelocityCalculator(void) {
  rightLinearVelocity = (RF.desiredDirection * RF.getCurrentVelocity() + RM.desiredDirection * RM.getCurrentVelocity() + RB.desiredDirection * RB.getCurrentVelocity()) * radius * piRad;
  leftLinearVelocity = (LF.desiredDirection * LF.getCurrentVelocity() + LM.desiredDirection * LM.getCurrentVelocity() + LB.desiredDirection * LB.getCurrentVelocity()) * radius * piRad;

  linearVelocity = (rightLinearVelocity - leftLinearVelocity)  / 6;
  rotationalVelocity = (leftLinearVelocity + rightLinearVelocity) / wheelBase;
}

// Toggle 0-5 motors
void toggleMotors(bool turnMotorOn) {
  if (turnMotorOn) {
    Cmds.isActivated = true;
  }
  else {
    Cmds.isActivated = false;
  }
}

// Emergency stop all motors
void stopMotors(void) {
  DcMotor::velocityHandler(*motorPtrList,0, 0); // Set all motors throttle and steering to 0
}

// Close motors loop
void closeMotorsLoop(void) {
  // Stop rover first
  if (Cmds.isActivated) {
    stopMotors();
  }

  toggleEncoder(true);
  maxOutputSignal = MAX_RPM_VALUE; 
  minOutputSignal = MIN_RPM_VALUE;

  // Set motors 0-5 open loop off
  for (i = 0; i < NUM_MOTORS; i++) {
    motorPtrList[i]->isOpenLoop = false;
  }
}

// Open motors loop
void openMotorsLoop(void) {
  // Stop rover first
  if (Cmds.isActivated) {
    stopMotors();
  }

  maxOutputSignal = MAX_PWM_VALUE; 
  minOutputSignal = MIN_PWM_VALUE;

  // Set motors 0-5 open loop on
  for (i = 0; i < NUM_MOTORS; i++) {
    motorPtrList[i]->isOpenLoop = true; 
  }
}

// Toggle joystick
void toggleJoystick(bool turnJoystickOn) {
  if (turnJoystickOn) {
    if (Cmds.isActivated) {
      stopMotors();
      Cmds.isJoystickMode = true;
    }
    else if (!Cmds.isActivated) {
      Cmds.isJoystickMode = true;
    }
  }
  else {
    if (Cmds.isActivated) {
      stopMotors();
      Cmds.isJoystickMode = false;
    }
    else if (!Cmds.isActivated) {
      Cmds.isJoystickMode = false;
    }
  }
}

// Toggle gps printing
void toggleGps(bool turnGpsOn) {
  if (turnGpsOn) {
    Cmds.isGpsImu = true;
  }
  else {
    Cmds.isGpsImu = false;
  }
}

// Toggle speed printing
void toggleEncoder(bool turnEncOn) {
  if (turnEncOn) {
    if (Cmds.isActivated) {
      Cmds.isEnc = true;

    }
    else if (!Cmds.isActivated) {
      Cmds.isEnc = true;
    }
  }
  else {
    Cmds.isEnc = false;
  }
}

// Toggle acceleration limiter
void toggleAcceleration(bool turnAccelOn) {
  if (turnAccelOn) {
    for (i = 0; i < NUM_MOTORS; i++) {
        motorPtrList[i]->accLimit = true;
    }
  }
  else {
    for (i = 0; i < NUM_MOTORS; i++) {
        motorPtrList[i]->accLimit = false;
    }
  }
}

// Print rover status (active or not)
void getRoverStatus(void) {
  int statusMessages = 9;
  String msgs[statusMessages];
  msgs[0] = "ASTRO Astro has " + String(NUM_MOTORS) + " motors";
  msgs[1] = "ASTRO Wheels: " + String(Cmds.isActivated ? "ACTIVE" : "INACTIVE");
  msgs[2] = "ASTRO Steering: " + String(Cmds.isSteering ? "Steering Control" : "Motor Control");
  msgs[3] = "ASTRO Encoders: " + String(Cmds.isEnc ? "ON" : "OFF");
  msgs[4] = "ASTRO GPS " + String(Cmds.gpsError ? "ERROR: " + Cmds.gpsErrorMsg : "Success");
  msgs[5] = "ASTRO IMU " + String(Cmds.imuError ? "ERROR: " + Cmds.imuErrorMsg : "Success");
  msgs[6] = "ASTRO Nav Stream: " + String(Cmds.isGpsImu ? "ON" : "OFF");
  
  msgs[7] += "ASTRO Motor loop statuses: ";
  for (int i = 0; i < NUM_MOTORS; i++) { //6 is hardcoded, should be using a macro
    msgs[7] += String(motorPtrList[i]->isOpenLoop ? "Open" : "CLose");
    if (i != NUM_MOTORS - 1) msgs[7] += ", ";
  }

  msgs[8] += "ASTRO Motor accel: ";
  for (int i = 0; i < NUM_MOTORS; i++) { //6 is hardcoded, should be using a macro
    msgs[8] += (motorPtrList[i]->accLimit) ? "ON" : "OFF";
    if (i != NUM_MOTORS - 1) msgs[8] += ", ";
  }

  for (int i = 0; i < statusMessages; i++) {
    commandCenter->sendDebug(msgs[i].c_str());
  }

}

// Throttle -49 to 49 and Steering -49 to 49
void moveRover(int8_t roverThrottle, int8_t roverSteering) {
  if (!Cmds.isActivated) {
    commandCenter->sendDebug("ASTRO Astro isn't activated yet!");
  }
  else {
    throttle = (float) roverThrottle; // From Globals.h
    steering = (float) roverSteering; // From Globals.h

    // Set wheel throttle and steering
    DcMotor::velocityHandler(getMotorList(),throttle, steering); 

    // Displayed from globals.h
    Cmds.sinceThrottle = 0;
  }
} 

// Wheel number 0 to 5 and -255 to 255 
void moveWheel(uint8_t wheelNumber, int16_t wheelPWM) {
  if (!Cmds.isActivated) {
    commandCenter->sendDebug("ASTRO Astro isn't activated yet!");
  }
  else {
    motorNumber = (int) wheelNumber;
    int motorSpeed = (int) wheelPWM;
    int dir = 1;
    Cmds.sinceThrottle = 0;
    steering = 0; // From Globals.h

    if (motorSpeed < 0 ) {
      dir = - 1;
    }

    if (motorNumber >= 1 && motorNumber <= 6) {
      motorPtrList[motorNumber-1]->calcCurrentVelocity();
      motorPtrList[motorNumber-1]->setVelocity(dir , abs(motorSpeed), motorPtrList[motorNumber-1]->getCurrentVelocity());
    }
    else {
      commandCenter->sendDebug("ASTRO invalid motor  number");
    }
  }
}

//! Attach the servos to pins
void attachServos() {
  frontSide.attach(FS_SERVO);
  frontSide.write(SERVO_STOP);
  frontBase.attach(FB_SERVO);
  frontBase.write(FRONT_BASE_DEFAULT_PWM);

  rearSide.attach(RS_SERVO);
  rearSide.write(SERVO_STOP);
  rearBase.attach(RB_SERVO);
  rearBase.write(REAR_BASE_DEFAULT_PWM);
}

void initMotorEncoder0(void) {
  RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
  pinMode(RF.encoderPinB, INPUT_PULLUP);
  pinMode(RF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, CHANGE);
  RF.pidController.setGainConstants(kp, ki, kd);
}

void initMotorEncoder1(void) {
  RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
  pinMode(RM.encoderPinB, INPUT_PULLUP);
  pinMode(RM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, CHANGE);
  RM.pidController.setGainConstants(kp, ki, kd);
}

void initMotorEncoder2(void) {
  RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
  pinMode(RB.encoderPinB, INPUT_PULLUP);
  pinMode(RB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(kp, ki, kd);
}

void initMotorEncoder3(void) {
  LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
  pinMode(LF.encoderPinB, INPUT_PULLUP);
  pinMode(LF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, CHANGE);
  LF.pidController.setGainConstants(kp, ki, kd);
}

void initMotorEncoder4(void) {
  LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
  pinMode(LM.encoderPinB, INPUT_PULLUP);
  pinMode(LM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, CHANGE);
  LM.pidController.setGainConstants(kp, ki, kd);
}

void initMotorEncoder5(void) {
  LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
  pinMode(LB.encoderPinB, INPUT_PULLUP);
  pinMode(LB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, CHANGE);
  LB.pidController.setGainConstants(kp, ki, kd);
}

void initSerialCommunications(void) {
  // Create serial connection with teensy pins 0 and 1
  commandCenter->startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN, ENABLE_PIN, 2);

  // initialize serial communications at 115200 bps:
  Serial.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
  Serial.setTimeout(SERIAL_TIMEOUT);
  delay(300); // NECESSARY. Give time for serial port to set up

  setMotorList(*motorPtrList);
  setServoList(*servoPtrList);
}

//! Initiate encoder for dcMotor objects and pinModes
void initEncoders(void) {
  initMotorEncoder0();
  initMotorEncoder1();
  initMotorEncoder2();
  initMotorEncoder3();
  initMotorEncoder4();
  initMotorEncoder5();
}

void rf_encoder_interrupt(void) {
  RF.dt += micros() - RF.prevTime;
  RF.prevTime = micros();
  RF.encoderCount++;
}

void rm_encoder_interrupt(void) {
  RM.dt += micros() - RM.prevTime;
  RM.prevTime = micros();
  RM.encoderCount++;
}

void rb_encoder_interrupt(void) {
  RB.dt += micros() - RB.prevTime;
  RB.prevTime = micros();
  RB.encoderCount++;
}

void lf_encoder_interrupt(void) {
  LF.dt += micros() - LF.prevTime;
  LF.prevTime = micros();
  LF.encoderCount++;
}

void lm_encoder_interrupt(void) {
  LM.dt += micros() - LM.prevTime;
  LM.prevTime = micros();
  LM.encoderCount++;
}

void lb_encoder_interrupt(void) {
  LB.dt += micros() - LB.prevTime;
  LB.prevTime = micros();
  LB.encoderCount++;
}

// Messages to get rover information
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371
void getLinearVelocity() {
  // Create message and send to OBC (Wheel Teensy to OBC)
  // CommandID set to 2 for sendLinearVelocity
  byte *linearVelocityByte = (byte *)&linearVelocity;
  internal_comms::Message* message = commandCenter->createMessage(
      2, sizeof(linearVelocityByte), linearVelocityByte);
  commandCenter->sendMessage(*message);
}

void getRotationalVelocity() {
  // Create message and send to OBC (Wheel Teensy to OBC)
  // CommandID set to 3 for sendRotationalVelocity
  byte *rotationalVelocityByte = (byte *)&rotationalVelocity;
  internal_comms::Message* message = commandCenter->createMessage(
      3, sizeof(rotationalVelocityByte), (byte*)rotationalVelocityByte);
  commandCenter->sendMessage(*message);
}

void getCurrentVelocity() {
  float currentVelocities[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    currentVelocities[i] = motorPtrList[i]->getCurrentVelocity();
  }
  // Create message and send to OBC (Wheel Teensy to OBC)
  // CommandID set to 4 for sendCurrentVelocity
  internal_comms::Message* message = commandCenter->createMessage(
      4, sizeof(currentVelocities), (byte*)currentVelocities);
  commandCenter->sendMessage(*message);
}

void getDesiredVelocity() {
  float desiredVelocities[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    desiredVelocities[i] = motorPtrList[i]->desiredVelocity;
  }
  // Create message and send to OBC (Wheel Teensy to OBC)
  // CommandID set to 5 for sendDesiredVelocity
  internal_comms::Message* message = commandCenter->createMessage(
      5, sizeof(desiredVelocities), (byte*)desiredVelocities);
  commandCenter->sendMessage(*message);
}

void getBatteryVoltage() {
  float vsense = analogRead(V_SENSE_PIN);
  vsense *= 0.003225806; //convert to 3.3V reference from analog values (3.3/1023=0.003225806)
  float vbatt = vsense * 6.0;

  // Create message and send to OBC (Wheel Teensy to OBC)
  // CommandID set to 6 for vbatt
  byte *vbattByte = (byte *)&vbatt;
  internal_comms::Message* message = commandCenter->createMessage(
      6, sizeof(vbattByte), (byte*)vbattByte);
  commandCenter->sendMessage(*message);
}

void pingWheels() {
  // Create message and send to OBC (Wheel Teensy to OBC)
  // CommandID set to 69 for ping
  internal_comms::Message* message = commandCenter->createMessage(69, 0, nullptr);
  commandCenter->sendMessage(*message);
}



