
#include "Commands.h"
#include "navigation.h"

#include <SoftwareSerial.h>
/*
choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
should work.
*/
//#define DEVEL_MODE_1 1
////#define DEVEL_MODE_2 2
//
//#if defined(DEVEL_MODE_1)
//// serial communication over usb
//#define UART_PORT Serial
//#elif defined(DEVEL_MODE_2)
//// serial communication over uart with odroid, teensy plugged into pcb and odroid
//#define UART_PORT Serial1
//#endif



SoftwareSerial bluetooth(9, 10);
ArduinoBlue phone(bluetooth);

elapsedMillis sinceFeedbackPrint; // timer for sending motor speeds and battery measurements
elapsedMillis sinceLedToggle; // timer for heartbeat
elapsedMillis sinceSensorRead; // timer for reading battery, gps and imu data
elapsedMillis sinceMC; // timer for reading battery, gps and imu data


float wheelBase = 0.33; //distance between left and right wheels
float radius = 0.14; // in m
float piRad = 0.10472; // Pi in radians
//float kp = 23.5;
float kp = 14.1;
float ki = 0.282;
float kd = 40.625;
//float kp = 10.0;
//float ki = 0.02;
//float kd = 0.05;
float linearVelocity, rotationalVelocity, rightLinearVelocity, leftLinearVelocity;
String rotation; // Rotation direction of the whole rover


// constructors
DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO, "Front Right Motor");
DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO, "Middle Right Motor");
DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO, "Rear Right Motor");
DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO, "Front Left Motor");
DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO, "Middle Left Motor");
DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO, "Rear Left Motor");

DcMotor motorList[] = {RF, RM, RB, LF, LM, LB};

Servo frontSide, frontBase, rearSide, rearBase;
Servo servoList[] = {frontSide, frontBase, rearSide, rearBase};
Commands Cmds;
/* function declarations */
// initializers
void attachServos(void); // attach pins to servo objects
void initEncoders(void); // Encoder initiation (attach interrups and pinModes for wheel encoders
void serialHandler(void); // Read String that automatically listens to all available ports and bluetooth. If uart is availble and reads who, its will switch devMode to false
void initPids(void); // Initiate PID for DMotor
// during operation
void roverVelocityCalculator(void);


// encoder interrupts
void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);

void setup() {
  // initialize serial communications at 115200 bps:
  Serial.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
  Serial1.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial1.setTimeout(SERIAL_TIMEOUT);
  bluetooth.begin(9600);
  bluetooth.setTimeout(50);
  delay(300); // NECSSARY. Give time for serial port to set up

  devMode = true; //if devMode is true then connection is through usb serial
 
  Cmds.setBluetooth(&bluetooth);
  Cmds.setMotorList(motorList);
  Cmds.setServoList(servoList);
  initPins();
  initEncoders();
  initPids();
  attachServos();
  initNav(Cmds);
  if (Cmds.isOpenLoop) {
    maxOutputSignal = MAX_PWM_VALUE;
    minOutputSignal = MIN_PWM_VALUE;
  }
  else {
    maxOutputSignal = MAX_RPM_VALUE;
    minOutputSignal = MIN_RPM_VALUE;
  }
  Cmds.setupMessage();
}

void loop() {
  // incoming format example: "5: 7"
  // this represents the speed for throttle:steering
  // as well as direction by the positive/negative sign

  serialHandler();

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
  }

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

//! Figures out which serial being used
void serialHandler(void) {
  if (devMode) {
    if (Serial1.available()) {
      cmd = Serial1.readStringUntil('\n');
      cmd.trim();
      Helpers::get().ser_flush();
      if (cmd == "who") {
        devMode = false ;
      }
      Cmds.handler(cmd, "UART");
    }
    else if (Serial.available()) {
      cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "who") {
         devMode = true;
      }
      Cmds.handler(cmd, "USB");
    }
    else if (bluetooth.available() && Cmds.bluetoothMode) {
      Cmds.bleHandler();
    }
  }
  else {
    if (Serial1.available()) {
      cmd = Serial1.readStringUntil('\n');
      cmd.trim();
      Helpers::get().ser_flush();
      Cmds.handler(cmd, "UART");
    }
    else if (Cmds.bluetoothMode) {
      Cmds.bleHandler();
    }
  }

}

//! attach the servos to pins
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

//! Initiate encoder for dcMotor objects and pinModes
void initEncoders(void) {
  RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
  pinMode(RF.encoderPinB, INPUT_PULLUP);
  pinMode(RF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, CHANGE);
  RF.pidController.setGainConstants(kp, ki, kd);
  //    RF.pidController.setGainConstants(3.15, 0.0002, 0.0);

  RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
  pinMode(RM.encoderPinB, INPUT_PULLUP);
  pinMode(RM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, CHANGE);
  //    RM.pidController.setGainConstants(3.15, 0.0002, 0.0);
  RM.pidController.setGainConstants(kp, ki, kd);

  RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
  pinMode(RB.encoderPinB, INPUT_PULLUP);
  pinMode(RB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(kp, ki, kd);
  //    RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
  pinMode(LF.encoderPinB, INPUT_PULLUP);
  pinMode(LF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, CHANGE);
  LF.pidController.setGainConstants(kp, ki, kd);
  //    LF.pidController.setGainConstants(kp, 0.0002, 0.0);

  LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
  pinMode(LM.encoderPinB, INPUT_PULLUP);
  pinMode(LM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, CHANGE);
  LM.pidController.setGainConstants(kp, ki, kd);
  //    LM.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
  pinMode(LB.encoderPinB, INPUT_PULLUP);
  pinMode(LB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, CHANGE);
  LB.pidController.setGainConstants(kp, ki, kd);
  //    LB.pidController.setGainConstants(3.15, 0.0002, 0.0);
}

//! Initiate PID objects for Dc Motors
void initPids(void) {
  /*
  RF.pidController.setJointVelocityTolerance(2.0 * RF.gearRatioReciprocal);
  RM.pidController.setJointVelocityTolerance(2.0 * RM.gearRatioReciprocal);
  RB.pidController.setJointVelocityTolerance(2.0 * RB.gearRatioReciprocal);

  LF.pidController.setJointVelocityTolerance(2.0 * LF.gearRatioReciprocal);
  LM.pidController.setJointVelocityTolerance(2.0 * LM.gearRatioReciprocal);
  LB.pidController.setJointVelocityTolerance(2.0 * LB.gearRatioReciprocal);

  RF.pidController.setOutputLimits(-50, 50, 5.0);
  RM.pidController.setOutputLimits(-50, 50, 5.0);
  RB.pidController.setOutputLimits(-50, 50, 5.0);

  LF.pidController.setOutputLimits(-50, 50, 5.0);
  LM.pidController.setOutputLimits(-50, 50, 5.0);
  LB.pidController.setOutputLimits(-50, 50, 5.0);
  */
}

void rf_encoder_interrupt(void) {
  RF.dt += micros() - RF.prevTime;
  RF.prevTime = micros();
  RF.encoderCount++;
  //    motorList[0].setVelocity(1 , 0, motorList[0].getCurrentVelocity());

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
  //    Serial.println(LF.dt);
  //    Serial.println(LF.encoderCount);
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
