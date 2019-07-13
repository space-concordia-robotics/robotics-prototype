
#include "ArduinoBlue.h"
#include "DcMotor.h"
#include <SoftwareSerial.h>
#include <Servo.h>

//GPS & IMU Includes
#include <SparkFun_I2C_GPS_Arduino_Library.h>
I2CGPS myI2CGPS;  // I2C object
#include "TinyGPS++.h"
TinyGPSPlus gps;   // GPS object
#include <Wire.h>
#include <LSM303.h>  // contains a sketch for calibrating
LSM303 compass;
//#define TEST 1

/* comms */

#define FEEDBACK_PRINT_INTERVAL 50
#define LED_BLINK_INTERVAL 1000
elapsedMillis sinceFeedbackPrint; // timer for sending motor speeds and battery measurements
elapsedMillis sinceLedToggle; // timer for heartbeat
unsigned long prevRead = millis(); // timer for reading commands
unsigned long prevReadNav = millis(); // timer for reading nav data
String cmd;

void blePrint(String cmd);
void blePrintln(String cmd);
void blePrintres(String cmd);
void ser_flush(void);
void toggleLed();
void toggleLed2();
String getValue(String data, char separator, int index);

// Bluetooth connection TX of bluetooth goes to pin 9 and RX of bluetooth goes to pin 10
SoftwareSerial bluetooth(9, 10);
ArduinoBlue phone(bluetooth);

// Modes of operation
// Bluetooth control - basestation control - PID - Open Loop

//boolean isActivated = false;
//boolean isActivated = false;
//boolean isOpenloop = true; // No PID controller
//boolean bluetoothMode = true; // changed to false - JOSH
//boolean joystickMode = true;

/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
//#define DEVEL_MODE_1 1
#define DEVEL_MODE_2 2

#if defined(DEVEL_MODE_1)
// serial communication over uart with odroid, teensy plugged into pcb and odroid
#define UART_PORT Serial
#define PRINT(a) Serial.print(a)
#define PRINTln(a) Serial.println(a)
#define PRINTRES(a,b) Serial.print(a, b)
/*
  #define PRINT(a) Serial.print(a);  blePrint(a);
  #define PRINTln(a) Serial.println(a); blePrintln(a);
  #define PRINTRES(a,b) Serial.print(a, b);  blePrintres(a, b);
*/
#elif defined(DEVEL_MODE_2)
#define UART_PORT Serial1
#define PRINT(a) Serial1.print(a)
#define PRINTln(a) Serial1.println(a)
#define PRINTRES(a,b) Serial1.print(a, b)
/*
  #define PRINT(a) Serial1.print(a);  blePrint(a);
  #define PRINTln(a) Serial1.println(a); blePrintln(a);
  #define PRINTRES(a,b) Serial1.print(a, b); blePrintres(a ,b);
*/
#endif

#define V_SENSE_PIN 39 // for reading battery voltage

/* camera servos */
#define FS_SERVO 22
#define FB_SERVO 23
#define TS_SERVO 16
#define TB_SERVO 17
Servo frontSide; Servo frontBase;
Servo topSide; Servo topBase;

/* wheel motors */
#define PULSES_PER_REV      14
#define maxRpm              30
#define GEAR_RATIO         188.61

// F -> Front, B -> Back,  M -> Middle, L -> Left, R -> Right,
// DIR -> Direction pin, PWM -> Signal Pin, EA ->Encoder A, EB -> Encoder B
// DC Motor naming: Positing_purpose-pin, eg RF_PWM or LM_DIR

// drivers
#define RF_DIR   2
#define RM_DIR   11
#define RB_DIR   12
#define LF_DIR   24
#define LM_DIR   25
#define LB_DIR   26

#define RF_PWM   3
#define RM_PWM   4
#define RB_PWM   5
#define LF_PWM   6
#define LM_PWM   7
#define LB_PWM   8

// encoders
#define RF_EA    27
#define RF_EB    28
#define RM_EA    31
#define RM_EB    32
#define RB_EA    29
#define RB_EB    30

#define LF_EA    37
#define LF_EB    38
#define LM_EA    36
#define LM_EB    35
#define LB_EA    33
#define LB_EB    34

// constructors
DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO, "Front Right Motor");
DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO, "Middle Right Motor");
DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO, "Rear Right Motor");

DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO, "Front Left Motor");
DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO, "Middle Left Motor");
DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO, "Rear Left Motor");

DcMotor motorList[] = {RF, RM, RB, LF, LM, LB};

/* more variables */
float throttle = 0, steering = 0, heading = 0; // Input values for set velocity functions
int loop_state, button, i; // Input values for set velocity functions
float deg = 0;  // steering ratio between left and right wheel
int maxInputSignal = 49;  // maximum speed signal from controller
int minInputSignal = -49; // minimum speed signal from controller
int maxOutputSignal = 255;
int minOutputSignal = -255;
int motorNumber;
int leftMotorDirection; // CCW =1 or CW =-1
int rightMotorDirection; // CCW =1 or CW =-1
float desiredVelocityRight = 0;
float desiredVelocityLeft = 0;
float d = 0.33; //distance between left and right wheels
float radius = 14;
float forwardVelocity, rotationalVelocity, rightLinearVelocity, leftLinearVelocity;
String rotation; // Rotation direction of the whole rover

/* more functions */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void initPins(void); // Initiate pinMode for direction and pwm pins for cytron drivers
void initEncoders(void);    // Encoder initiation (attach interrups and pinModes for wheel encoders
void initPids(void);    // Initiate PID for DMotor
void initNav(void);

void navHandler(void);
void velocityHandler(float throttle, float steering);
void roverVelocityCalculator(void);
void motor_encoder_interrupt(int motorNumber);

void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);

/* more comms */
#include "Vsense.h"
#include "commands.h"
Commands Commands;

void setup() {
  // initialize serial communications at 115200 bps:
  UART_PORT.begin(115200); // switched from 9600 as suggested to conform with the given gps library
  UART_PORT.setTimeout(20);
  bluetooth.begin(9600);
  bluetooth.setTimeout(50);

  delay(50);  // do not print too fast!
  //ser_flush();
  initPins();
  initEncoders();
  initPids();
  initNav();
  delay(300);
  Commands.setupMessage();
}

void loop() {
  //if (millis() - prevRead > 30) {
    // incoming format example: "5:7"
    // this represents the speed for throttle:steering
    // as well as direction by the positive/negative sign
    // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
    if (UART_PORT.available()) {
      String cmd = UART_PORT.readStringUntil('\n');
      cmd.trim();
      ser_flush();
      Commands.handler(cmd, "Serial");
    }
    /*
      else if (bluetooth.available()) {
      Commands.bleHandler();
      }*/

    if (Commands.isActivated && Commands.bluetoothMode) {
      velocityHandler(throttle, steering);
      if (Commands.isEnc) {
        // roverVelocityCalculator();
        RF.calcCurrentVelocity();
        RM.calcCurrentVelocity();
        RB.calcCurrentVelocity();
        LF.calcCurrentVelocity();
        LM.calcCurrentVelocity();
        LB.calcCurrentVelocity();
        PRINTln("ASTRO ~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~.~");
        PRINT("ASTRO left: " + String(desiredVelocityLeft));
        PRINTln(" - right: " + String(desiredVelocityRight));
      }
    }
    prevRead = millis();
  //} // end of command listening

  if (millis() - prevReadNav > 200) {
    navHandler();
    prevReadNav = millis();
  }
  if (sinceLedToggle > LED_BLINK_INTERVAL) {
    toggleLed();
    vbatt_read();
    sinceLedToggle = 0;
  }
  if (sinceFeedbackPrint > FEEDBACK_PRINT_INTERVAL) {
    PRINT("ASTRO Motor Speeds: ");
    if(Commands.isEnc) {
      for (int i = 0; i < 6; i++) { //6 is hardcoded, should be using a macro
          PRINT(motorList[i].getCurrentVelocity());
          if (i != 5) PRINT(", ");
      }
      PRINTln("");
    }
    else {
      PRINT(String(desiredVelocityRight)+", ");
      PRINT(String(desiredVelocityRight)+", ");
      PRINT(String(desiredVelocityRight)+", ");
      PRINT(String(desiredVelocityLeft)+", ");
      PRINT(String(desiredVelocityLeft)+", ");
      PRINT(String(desiredVelocityLeft));
      PRINTln("");
    }
    sinceFeedbackPrint = 0;
  }
} // end of loop

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void velocityHandler(float throttle, float steering) {
  // If statement for CASE 1: steering toward the RIGHT
  if (steering > 0 ) {
    deg = mapFloat(steering, 0, maxInputSignal, 1, -1);
    desiredVelocityRight = mapFloat(throttle * deg, minInputSignal, maxInputSignal, minOutputSignal, maxOutputSignal);
    desiredVelocityLeft = mapFloat(throttle, minInputSignal, maxInputSignal,  minOutputSignal, maxOutputSignal);
    rotation = "CW";
  }
  // If statement for CASE 2: steering toward the LEFT or not steering
  if (steering <= 0 ) {
    deg = mapFloat(steering, minInputSignal, 0, -1, 1);
    desiredVelocityRight = mapFloat(throttle, minInputSignal, maxInputSignal, minOutputSignal, maxOutputSignal);
    desiredVelocityLeft = mapFloat(throttle * deg, minInputSignal, maxInputSignal, minOutputSignal, maxOutputSignal);
    rotation = "CCW";
  }
  if (desiredVelocityLeft < 0 ) {
    leftMotorDirection = 1;
  }
  else {
    leftMotorDirection = -1;
  }

  if (desiredVelocityRight < 0 ) {
    rightMotorDirection = -1;
  }
  else {
    rightMotorDirection = 1;
  }

  RF.calcCurrentVelocity();
  RM.calcCurrentVelocity();
  RB.calcCurrentVelocity();
  LF.calcCurrentVelocity();
  LM.calcCurrentVelocity();
  LB.calcCurrentVelocity();

  RF.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RF.getCurrentVelocity());
  RM.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RM.getCurrentVelocity());
  RB.setVelocity(rightMotorDirection, abs(desiredVelocityRight), RB.getCurrentVelocity());
  LF.setVelocity(rightMotorDirection, abs(desiredVelocityLeft), LF.getCurrentVelocity());
  LM.setVelocity(rightMotorDirection, abs(desiredVelocityLeft), LM.getCurrentVelocity());
  LB.setVelocity(rightMotorDirection, abs(desiredVelocityLeft), LB.getCurrentVelocity());
}
void roverVelocityCalculator(void) {
  rightLinearVelocity = (RF.getDirection() * RF.getCurrentVelocity() + RM.getDirection() * RM.getCurrentVelocity() + RB.getDirection() * RB.getCurrentVelocity()) * radius * 0.10472;
  leftLinearVelocity = (LF.getDirection() * LF.getCurrentVelocity() + LM.getDirection() * LM.getCurrentVelocity() + LB.getDirection() * LB.getCurrentVelocity()) * radius * 0.10472;

  forwardVelocity = (rightLinearVelocity + leftLinearVelocity)  / 6;
  rotationalVelocity = (leftLinearVelocity - rightLinearVelocity) / d;
}

void ser_flush(void) {
  while (UART_PORT.available()) {
    UART_PORT.read();
  }
}
void blePrint(String cmd) {
  if (Commands.bluetoothMode) {
    bluetooth.print(cmd);
    delay(50);
  }
}
void blePrintln(String cmd) {
  if (Commands.bluetoothMode) {
    bluetooth.println(cmd);
    delay(50);
  }
}
void blePrintres(float a, float b) {
  if (Commands.bluetoothMode) {
    bluetooth.print(a, b);
    delay(50);
  }
}
//! Parse data for throttle and steering variables
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void toggleLed() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
void toggleLed2() {
  /*
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(350);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(150);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  */
}

void initPins(void) {
  pinMode(RF_DIR, OUTPUT);
  pinMode(RF_PWM, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_PWM, OUTPUT);
  pinMode(RB_DIR, OUTPUT);
  pinMode(RB_PWM, OUTPUT);

  pinMode(LF_DIR, OUTPUT);
  pinMode(LF_PWM, OUTPUT);
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_PWM, OUTPUT);
  pinMode(LB_DIR, OUTPUT);
  pinMode(LB_PWM, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(V_SENSE_PIN, INPUT);

  frontSide.attach(FS_SERVO);
  frontBase.attach(FB_SERVO);
  topSide.attach(TS_SERVO);
  topBase.attach(TB_SERVO);
}
//! Initiate encoder for dcMotor objects and pinModes
void initEncoders(void) {
  RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
  pinMode(RF.encoderPinB, INPUT_PULLUP);
  pinMode(RF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
  pinMode(RM.encoderPinB, INPUT_PULLUP);
  pinMode(RM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
  pinMode(RB.encoderPinB, INPUT_PULLUP);
  pinMode(RB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
  pinMode(LF.encoderPinB, INPUT_PULLUP);
  pinMode(LF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
  pinMode(LM.encoderPinB, INPUT_PULLUP);
  pinMode(LM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);

  LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
  pinMode(LB.encoderPinB, INPUT_PULLUP);
  pinMode(LB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(3.15, 0.0002, 0.0);
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
void initNav(void) {
  myI2CGPS.begin(Wire, 400000);
    if (myI2CGPS.begin(Wire, 400000) == false) { // Wire corresponds to the SDA1,SCL1 on the Teensy 3.6 (pins 38,37)
    Commands.error = true;
    Commands.errorMessage = "ASTRO GPS wires aren't connected properly, please reboot\n";
    Commands.errorMsg();
    }

    else if (!Commands.error) {
    compass.init();
    compass.enableDefault();
    compass.setTimeout(100);
    compass.m_min = (LSM303::vector <int16_t>) {
      -1794, +1681, -2947
    };
    compass.m_max = (LSM303::vector <int16_t>) {
      +3359, +6531, +2016
    };
  }
}
//min: { -1794,  +1681,  -2947 }    max: { +3359,  +6531,  +2016 }

void navHandler(void) {
  if (!Commands.error && Commands.isGpsImu) {
    bool gotGps = false;
    if (myI2CGPS.available()) {         // returns the number of available bytes from the GPS module
      gps.encode(myI2CGPS.read());       // Feeds the GPS parser
      if (gps.time.isUpdated()) {        // Checks to see if new GPS info is available
        if (gps.location.isValid()) { // checks if valid location data is available
          gotGps = true;
        }
      }
    }

    compass.read();
    heading = compass.heading(LSM303::vector<int> { -1, 0, 0 });
    if (compass.timeoutOccurred()) {
      PRINT("ASTRO HEADING-N/A");
    }
    else {
      PRINT("ASTRO HEADING-OK ");
      PRINT(heading);
    }
    PRINT(" -- ");

    if (!gotGps) {
      PRINTln("GPS-N/A");
    }
    else {
      PRINT("GPS-OK ");
      PRINTRES(gps.location.lat(), 6); // print the latitude with 6 digits after the decimal
      PRINT(" "); // space
      PRINTRES(gps.location.lng(), 6); // print the longitude with 6 digits after the decimal
      PRINTln("");
    }
  }
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
