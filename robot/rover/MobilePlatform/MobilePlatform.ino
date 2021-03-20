//
// Edited by Michael on 2021-03-20.
//

#include "Navigation.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include "ArduinoBlue.h"

// Issue made to change includes to include
#include "includes/PinSetup.h"
#include "../internal_comms/include/CommandCenter.h"
#include "../internal_comms/include/Serial.h"
#include "includes/commands/WheelsCommandCenter.h"    
#include "includes/Globals.h"
#include "includes/Commands.h"  // This automatically includes DcMotor.h
#include "includes/Helpers.h"

/* Global variables*/
elapsedMillis sinceFeedbackPrint; // timer for sending motor speeds and battery measurements
elapsedMillis sinceLedToggle; // timer for heartbeat
elapsedMillis sinceSensorRead; // timer for reading battery, gps and imu data
elapsedMillis sinceMC; // timer for reading battery, gps and imu data

const float wheelBase = 0.33; //distance between left and right wheels
const float radius = 0.14; // in m
const float piRad = 0.10472; // Pi in radians
const float kp = 14.1;
const float ki = 0.282;
const float kd = 40.625;
float linearVelocity, rotationalVelocity, rightLinearVelocity, leftLinearVelocity;
String rotation; // Rotation direction of the whole rover

// Pins for Serial
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t TX_TEENSY_3_6_PIN = 1;

// Pins for Serial1
const uint8_t RX_TEENSY_3_6_PIN_SERIAL1 = 9;
const uint8_t TX_TEENSY_3_6_PIN_SERIAL1 = 10;

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

// Use to call commands 
Commands Cmds;

// To read commands from wheelscommandcenter
internal_comms::CommandCenter* commandCenter = new WheelsCommandCenter();
internal_comms::CommandCenter* commandCenter1 = new WheelsCommandCenter();

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

// Initialization Serial and Serial1
void initSerialCommunications(void);

// Wheel command methods from WheelsCommandCenter.cpp
void setMotorList(DcMotor* motorList);
void setServoList(Servo* servoList);
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
  Cmds.setupMessage();
}

// Running the wheels
void loop() {
  /*
  Choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
  */
  // Acquire method based on command sent from serial
  if (Serial.available()) {
    // Pointer to select the method (WheelsCommandCenter.cpp) to run based on the command
    internal_comms::readCommand(commandCenter); 
  }
  else if (Serial1.available()) {
    // Pointer to select the method (WheelsCommandCenter.cpp) to run based on the command
    internal_comms::readCommand(commandCenter1); 
  }

  if (sinceSensorRead > SENSOR_READ_INTERVAL) {
    Helpers::get().vbatt_read(V_SENSE_PIN);
    navHandler(Cmds);
    sinceSensorRead = 0;
  }

  if (sinceLedToggle > LED_BLINK_INTERVAL) {
    Helpers::get().toggleLed();
    sinceLedToggle = 0;
  }

  if (Cmds.sinceThrottle > THROTTLE_TIMEOUT && Cmds.throttleTimeOut) Cmds.stop(true);

  if (sinceMC > MOTOR_CONTROL_INTERVAL && Cmds.isActivated) {
    RF.calcCurrentVelocity();
    RM.calcCurrentVelocity();
    RB.calcCurrentVelocity();
    LF.calcCurrentVelocity();
    LM.calcCurrentVelocity();
    LB.calcCurrentVelocity();

    RF.setVelocity(RF.desiredDirection, fabs(RF.desiredVelocity), RF.getCurrentVelocity());
    RM.setVelocity(RM.desiredDirection, fabs(RM.desiredVelocity), RM.getCurrentVelocity());
    RB.setVelocity(RB.desiredDirection, fabs(RB.desiredVelocity), RB.getCurrentVelocity());
    LF.setVelocity(LF.desiredDirection, fabs(LF.desiredVelocity), LF.getCurrentVelocity());
    LM.setVelocity(LM.desiredDirection, fabs(LM.desiredVelocity), LM.getCurrentVelocity());
    LB.setVelocity(LB.desiredDirection, fabs(LB.desiredVelocity), LB.getCurrentVelocity());

    sinceMC = 0;
  } // End of loop

  if (sinceFeedbackPrint > FEEDBACK_PRINT_INTERVAL && Cmds.isActivated) {
    if (Cmds.isEnc) {
      Helpers::get().print("ASTRO Motor Speeds: ");
      Helpers::get().print(RF.getCurrentVelocity());
      Helpers::get().print(", ");
      Helpers::get().print(RM.getCurrentVelocity());
      Helpers::get().print(", ");
      Helpers::get().print(RB.getCurrentVelocity());
      Helpers::get().print(", ");
      Helpers::get().print(LF.getCurrentVelocity());
      Helpers::get().print(", ");
      Helpers::get().print(LM.getCurrentVelocity());
      Helpers::get().print(", ");
      Helpers::get().println(LB.getCurrentVelocity());

      roverVelocityCalculator();

      Helpers::get().print("ASTRO Desired Velocities: ");
      Helpers::get().print(String(RF.desiredVelocity) + ", ");
      Helpers::get().print(String(RM.desiredVelocity) + ", ");
      Helpers::get().print(String(RB.desiredVelocity) + ", ");
      Helpers::get().print(String(LF.desiredVelocity) + ", ");
      Helpers::get().print(String(LM.desiredVelocity) + ", ");
      Helpers::get().println(String(LB.desiredVelocity));
    }
    else {
      Helpers::get().print("ASTRO Motor Speeds: ");
      Helpers::get().print(String(RF.desiredVelocity) + ", ");
      Helpers::get().print(String(RM.desiredVelocity) + ", ");
      Helpers::get().print(String(RB.desiredVelocity) + ", ");
      Helpers::get().print(String(LF.desiredVelocity) + ", ");
      Helpers::get().print(String(LM.desiredVelocity) + ", ");
      Helpers::get().println(String(LB.desiredVelocity));
    }
    sinceFeedbackPrint = 0;
  }
}

void roverVelocityCalculator(void) {
  rightLinearVelocity = (RF.desiredDirection * RF.getCurrentVelocity() + RM.desiredDirection * RM.getCurrentVelocity() + RB.desiredDirection * RB.getCurrentVelocity()) * radius * piRad;
  leftLinearVelocity = (LF.desiredDirection * LF.getCurrentVelocity() + LM.desiredDirection * LM.getCurrentVelocity() + LB.desiredDirection * LB.getCurrentVelocity()) * radius * piRad;

  linearVelocity = (rightLinearVelocity - leftLinearVelocity)  / 6;
  rotationalVelocity = (leftLinearVelocity + rightLinearVelocity) / wheelBase;

  Helpers::get().print("ASTRO ");
  Helpers::get().print("Linear Velocity: ");
  Helpers::get().print(linearVelocity);
  Helpers::get().print(" m / s ");

  Helpers::get().print("Rotational Velocity: ");
  Helpers::get().print(rotationalVelocity);
  Helpers::get().println(" m ^ 2 / 6 ");
}

void setMotorList(DcMotor* motorList){
  Cmds.motorList = motorList;
}
void setServoList(Servo* servoList){
  Cmds.servoList = servoList;
}

DcMotor* getMotorList() {
  return Cmds.motorList;
}

// Toggle 0-5 motors
void toggleMotors(bool turnMotorOn) {
  if (turnMotorOn) {
    Cmds.isActivated = true;
    Helpers::get().println("ASTRO motors active!");
  }
  else {
    Cmds.isActivated = false;
    Helpers::get().println("ASTRO motors inactive!");
  }
}

// Emergency stop all motors
void stopMotors(void) {
  DcMotor::velocityHandler(motorList,0, 0); // Set all motors throttle and steering to 0
  // Not required to send as command to Navigation
}

// Close motors loop
void closeMotorsLoop(void) {
  // Stop rover first
  if (Cmds.isActivated) {
    stopMotors();
  }

  Helpers::get().println("ASTRO Turning encoders on first...");
  toggleEncoder(true);
  maxOutputSignal = MAX_RPM_VALUE; 
  minOutputSignal = MIN_RPM_VALUE;
  
  Helpers::get().println("ASTRO Bo!");

  // Set motors 0-5 open loop off
  for (i = 0; i < RobotMotor::numMotors; i++) {
    motorList[i].isOpenLoop = false;
    String msg = "ASTRO Motor " + String(i + 1);
    msg += String(" loop status is: ");
    msg += String(motorList[i].isOpenLoop ? "Open" : "Close");
    Helpers::get().println(msg);
  }
}

// Open motors loop
void openMotorsLoop(void) {
  // Stop rover first
  if (Cmds.isActivated) {
    stopMotors();
  }

  Helpers::get().println("ASTRO Turning encoders off first...");
  maxOutputSignal = MAX_PWM_VALUE; 
  minOutputSignal = MIN_PWM_VALUE;

  // Set motors 0-5 open loop on
  for (i = 0; i < RobotMotor::numMotors; i++) {
    motorList[i].isOpenLoop = true; 
    String msg = "ASTRO Motor " + String(i + 1);
    msg += String(" loop status is: ");
    msg += String(motorList[i].isOpenLoop ? "Open" : "Close");
    Helpers::get().println(msg);
  }
}

// Toggle joystick
void toggleJoystick(bool turnJoystickOn) {
  if (turnJoystickOn) {
    if (Cmds.isActivated) {
      stopMotors();
      Cmds.isJoystickMode = true;
      Helpers::get().println("ASTRO Joystick is active");
    }
    else if (!Cmds.isActivated) {
      Cmds.isJoystickMode = true;
      Helpers::get().println("ASTRO Joystick is active");
    }
  }
  else {
    if (Cmds.isActivated) {
      stopMotors();
      Cmds.isJoystickMode = false;
      Helpers::get().println("ASTRO ArduinoBlue Joystick is disabled");
    }
    else if (!Cmds.isActivated) {
      Cmds.isJoystickMode = false;
      Helpers::get().println("ASTRO ArduinoBlue Joystick is disabled");
    }
  }
}

// Toggle gps printing
void toggleGps(bool turnGpsOn) {
  if (turnGpsOn) {
    Cmds.isGpsImu = true;
    Helpers::get().println("ASTRO GPS and IMU Serial Stream is now Enabled");
  }
  else {
    Cmds.isGpsImu = false;
    Helpers::get().println("ASTRO GPS and IMU Serial Stream is now Disabled");
  }
}

// Toggle speed printing
void toggleEncoder(bool turnEncOn) {
  if (turnEncOn) {
    if (Cmds.isActivated) {
      Cmds.isEnc = true;
      Helpers::get().println("ASTRO Velocity Readings Stream from Motor Encoders is ON");
    }
    else if (!Cmds.isActivated) {
      Cmds.isEnc = true;
      Helpers::get().println("ASTRO Motor Velocity Reading Stream is ON but will start printing values once the Rover is activated");
      for (i = 0; i < RobotMotor::numMotors; i++) {
        Helpers::get().print("ASTRO Motor ");
        Helpers::get().print(i);
        Helpers::get().print(" current velocity: ");
        Helpers::get().println(motorList[i].getCurrentVelocity());
      }
    }
  }
  else {
    Cmds.isEnc = false;
    Helpers::get().println("ASTRO Velocity Readings Stream from Motor Encoders is OFF");
  }
}

// Toggle acceleration limiter
void toggleAcceleration(bool turnAccelOn) {
  if (turnAccelOn) {
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].accLimit = true;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += " Acceleration Limiter: ";
        msg += String(motorList[i].accLimit ? "Open" : "CLose");
        Helpers::get().println(msg);
    }
  }
  else {
    for (i = 0; i < RobotMotor::numMotors; i++) {
        motorList[i].accLimit = false;
        String msg = "ASTRO Motor " + String(i + 1);
        msg += " Acceleration Limiter: ";
        msg += String(motorList[i].accLimit ? "Open" : "CLose");
        Helpers::get().println(msg);
    }
  }
}

// Print rover status (active or not)
void getRoverStatus(void) {
  Helpers::get().println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
    Helpers::get().println("ASTRO Astro has " + String(RobotMotor::numMotors) + " motors");
    Helpers::get().println("ASTRO Wheels: " + String(Cmds.isActivated ? "ACTIVE" : "INACTIVE"));
    Helpers::get().println("ASTRO Steering: " + String(Cmds.isSteering ? "Steering Control" : "Motor Control"));
    Helpers::get().println("ASTRO Encoders: " + String(Cmds.isEnc ? "ON" : "OFF"));
    Helpers::get().println("ASTRO GPS " + String(Cmds.gpsError ? "ERROR: " + Cmds.gpsErrorMsg : "Success"));
    Helpers::get().println("ASTRO IMU " + String(Cmds.imuError ? "ERROR: " + Cmds.imuErrorMsg : "Success"));
    Helpers::get().println("ASTRO Nav Stream: " + String(Cmds.isGpsImu ? "ON" : "OFF"));
    Helpers::get().print("ASTRO Motor loop statuses: ");
    for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
      Helpers::get().print(String(motorList[i].isOpenLoop ? "Open" : "CLose"));
      if (i != RobotMotor::numMotors - 1) Helpers::get().print(", ");
    }
    Helpers::get().println("");

    Helpers::get().print("ASTRO Motor accel: ");
    for (int i = 0; i < RobotMotor::numMotors; i++) { //6 is hardcoded, should be using a macro
      Helpers::get().print((motorList[i].accLimit) ? "ON" : "OFF");
      if (i != RobotMotor::numMotors - 1) Helpers::get().print(", ");
    }
    Helpers::get().println("");

    Helpers::get().println("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
}

// Throttle -49 to 49 and Steering -49 to 49
void moveRover(int8_t roverThrottle, int8_t roverSteering) {
  if (!Cmds.isActivated) {
    Helpers::get().println("ASTRO Astro isn't activated yet!");
  }
  else {
    throttle = (float) roverThrottle; // From Globals.h
    steering = (float) roverSteering; // From Globals.h
    Helpers::get().println("ASTRO Throttle: " + String(throttle) + String(" -- Steering: ") + String(steering));

    // Set wheel throttle and steering
    DcMotor::velocityHandler(getMotorList(),throttle, steering); 

    // Displayed from globals.h
    String msg = "ASTRO left: " + String(desiredVelocityLeft);
    msg += " -- right: " + String(desiredVelocityRight);
    msg += " maxOutput: " + String(maxOutputSignal);
    Helpers::get().println(msg);
    Cmds.sinceThrottle = 0;
  }
} 

// Wheel number 0 to 5 and -255 to 255 
void moveWheel(uint8_t wheelNumber, int16_t wheelPWM) {
  if (!Cmds.isActivated) {
    Helpers::get().println("ASTRO Astro isn't activated yet!");
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
      motorList[motorNumber-1].calcCurrentVelocity();
      motorList[motorNumber-1].setVelocity(dir , abs(motorSpeed), motorList[motorNumber-1].getCurrentVelocity());
      Helpers::get().println("ASTRO " + String(motorList[motorNumber-1].motorName) + String("'s desired speed: ") + String(motorList[motorNumber-1].desiredVelocity) + String(" PWM "));
    }
    else {
      Helpers::get().println("ASTRO invalid motor  number");
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
  internal_comms::startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN);  // Start Serial
  internal_comms::startSerial(TX_TEENSY_3_6_PIN_SERIAL1, RX_TEENSY_3_6_PIN_SERIAL1); // Start Serial1

  // initialize serial communications at 115200 bps:
  Serial.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
  Serial1.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial1.setTimeout(SERIAL_TIMEOUT);
  delay(300); // NECESSARY. Give time for serial port to set up

  devMode = true; //if devMode is true then connection is through usb serial

  setMotorList(motorList);
  setServoList(servoList);
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
