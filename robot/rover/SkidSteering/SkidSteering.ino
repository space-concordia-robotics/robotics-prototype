//#include <ArduinoBlue.h>
#include "DcMotor.h"

//#define TEST 1

#define NO_PID 1 // PidController not active

#define PULSES_PER_REV      7
//#define GEAR_RATIO         71.16
#define maxVelocity         75
#define minVelocity         -75
#define GEAR_RATIO         188.61
float prevCurrent = 0;

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
#elif defined(DEVEL_MODE_2)
#define UART_PORT Serial4
#endif
#if defined(DEBUG_MODE) || defined(USER_MODE)
#define USE_TEENSY_HW_SERIAL 0 // this will make ArduinoHardware.h use hardware serial instead of usb serial
#endif

//SoftwareSerial bluetooth(9, 10);

//ArduinoBlue phone(bluetooth);
//-Gear Ratio
//-Encod pin a
//-Encod pin b
//- Pwm pin
//- dir pin
//- max velocity (RPM)
//- slowest speed (RPM)
//- current RPM
//- desired RPM

//- current velocity
//- desired velocity

// F -> Front, B -> Back,  M -> Middle, L -> Left, R -> Right,
// DIR -> Direction pin, PWM -> Signal Pin, EA ->Encoder A, EB -> Encoder B
// DC Motor naming: Positing_purpose-pin, eg RF_PWM or LM_DIR
// Types of Pins: DIR, PWM, Enc_A, Enc_B

// update later
#ifdef TEST
#define TEST_DIR   5
#define TEST_PWM   9
#define TEST_EA   2
#define TEST_EB   3

#endif
// Bluetooth connection TX of bluetooth goes to pin 9 and RX of bluetooth goes to pin 10
#define RF_DIR   14
#define RM_DIR   15
#define RB_DIR   16 // M4_A

#define LF_DIR   11
#define LM_DIR   24
#define LB_DIR   12

#define RF_PWM   2
#define RM_PWM   5
#define RB_PWM   6

#define LF_PWM   7
#define LM_PWM   8
#define LB_PWM   17 //M4_B

#define RF_EA    22
#define RF_EB    23

#define RM_EA    20
#define RM_EB    21

#define RB_EA    19
#define RB_EB    18

#define LF_EA    0
#define LF_EB    1

#define LM_EA    26
#define LM_EB    27

#define LB_EA    33
#define LB_EB    34



//
//
//#define RF_DIR   11
//#define RM_DIR   12
//#define RB_DIR   24
//
//#define LF_DIR   8
//#define LM_DIR   16
//#define LB_DIR   15
//
//#define RF_PWM   2
//#define RM_PWM   3
//#define RB_PWM   4
//
//#define LF_PWM   5
//#define LM_PWM   6
//#define LB_PWM   7
//
//#define RF_EA    25
//#define RF_EB    26
//
//#define RM_EA    27
//#define RM_EB    28
//
//#define RB_EA    29
//#define RB_EB    30
//
//#define LF_EA    31
//#define LF_EB    32
//
//#define LM_EA    33
//#define LM_EB    34
//
//#define LB_EA    35
//#define LB_EB    36

#ifdef TEST

DcMotor test(TEST_DIR, TEST_PWM, GEAR_RATIO);

#else
DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO);
DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO);
DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO);

DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO);
DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO);
DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO);

#endif
// This portion shall be removed once ROSyfied
int steeringShiftValue = 99;
int throttleShiftValue = 99;

float minThrottleController = 0;
float maxThrottleController = 0 + throttleShiftValue;
float minSteeringController = 0;
float maxSteeringController = 0 + steeringShiftValue;

int throttle, steering;

float deg = 0;
float rpm = 0;
int maxS = 49;
int minS = -49;
int leftMotorDirection;
int rightMotorDirection;
float v;

unsigned long startTime = millis();
unsigned long currTime = millis();
volatile unsigned long interruptCount = 0;
volatile unsigned long prevTime = micros();
volatile unsigned long dt;
float degAngularResolution = 1 / (float)(GEAR_RATIO*PULSES_PER_REV);

float desiredVelocityRight = 0;
float desiredVelocityLeft = 0;

// Right side servos
unsigned int prevRead = millis();

int prevThrottle = 0;
int prevSteering = 0;

float vy = 0;
float d = 0.33; //distance between left and right wheels
float vx, vth, vr, vl;

#define RANGE 255
#define REST  0
#define MAX_PWM REST + RANGE
#define MIN_PWM REST - RANGE

String rotation;
void velocityHandler(float throttle);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
boolean isActivated = false;

void encoder_ISR(void);
void test_encoder_interrupt(void);
void dcInterrupt(void);
void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);

IntervalTimer dcTimer;

void ser_flush(void) {
  while (UART_PORT.available()) {
    UART_PORT.read();
  }
}

void toggleLed() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


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

void initPins(void);
void initEncoders(void);
void initPids(void);

void setup() {
  // initialize serial communications at 9600 bps:
  UART_PORT.begin(9600);
  UART_PORT.setTimeout(50);
  //bluetooth.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  delay(1000);  // do not print too fast!
  toggleLed();
  delay(1000);
  toggleLed();
  UART_PORT.println("setup complete");
  ser_flush();

  //  pinMode(DC_PIN_FRONT_RIGHT, OUTPUT);
  //   // Write velocities for the Wheels on the RIGHT side
  //  analogWrite(DC_PIN_FRONT_RIGHT, 0);
  //
  //
  //  pinMode(DIR_PIN_FRONT_RIGHT, OUTPUT); delay(10);
  //
  //  // this sets up the initial rotation direction to counter clockwise, so High = CCW
  //  digitalWrite(DIR_PIN_FRONT_RIGHT, HIGH);
  //
  //  pinMode(M2_ENCODER_B, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_B), encoder_ISR, FALLING); delay(10);

#ifdef TEST
  pinMode(TEST_DIR, OUTPUT);
  pinMode(TEST_PWM, OUTPUT);
  test.attachEncoder(TEST_EA, TEST_EB, PULSES_PER_REV);
  pinMode(test.encoderPinB, INPUT_PULLUP);
  pinMode(test.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(test.encoderPinA), test_encoder_interrupt, RISING);
  //  attachInterrupt(digitalPinToInterrupt(test.encoderPinB), test_encoder/_interrupt, CHANGE);
  test.pidController.setGainConstants(1.0, 0.0, 0.0);

  //  test.pidController.setJointVelocityTolerance(2.0 * test.gearRatioRecipr/ocal);
#else
  //    DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO);
  //    DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO);
  //    DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO);
  //
  //    DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO);
  //    DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO);
  //    DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO);

  initPins();

  ser_flush();

  //    analogWrite(RF.getPwmPin(), 127);
  //    Serial.println(RF.getPwmPin());
  //

  //initEncoders();
  //initPids();

#endif

  dcTimer.begin(dcInterrupt, 20000);
}

void loop() {
  if ((millis() - prevRead > 1000)) {
    UART_PORT.println("hello there");
    prevRead = millis();
  }
  if (UART_PORT.available() && !isActivated) {
    toggleLed();
    String cmd = UART_PORT.readStringUntil('\n');
    ser_flush();

    UART_PORT.print("cmd: ");
    UART_PORT.println(cmd);

    if (cmd == "activate") {
      //toggleLed();
      isActivated = true;
    } else if (cmd == "who") {
      UART_PORT.println("rover");
    }
  }

  if ((millis() - prevRead > 20) && isActivated)
  {
    // incoming format example: "5:7"
    // this represents the speed for throttle;steering
    // as well as direction by the positive/negative sign
    String cmd = "";
    // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
    if (UART_PORT.available()) {
      toggleLed();
      //Serial.println("yooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo");
      // parse command
      cmd = UART_PORT.readStringUntil('\n');
      ser_flush();

      if (cmd == "who") {
        UART_PORT.println("rover");
      }

      if (cmd == "deactivate") {
        //toggleLed();
        isActivated = false;
      } else if (cmd.indexOf(":") > 0) {
        UART_PORT.println("Received command");
        throttle = getValue(cmd, ':', 0).toInt();
        steering = getValue(cmd, ':', 1).toInt();
        UART_PORT.print("TEENSY throttle: ");
        UART_PORT.println(throttle);
        UART_PORT.print("TEENSY steering: ");
        UART_PORT.println(steering);

        throttle -= 49.5;
        steering -= 49.5;
      }
    } else {
      //UART_PORT.println("No command received");
      throttle = 0;
      steering = 0;
    }

    //throttle = phone.getThrottle();
    //steering = phone.getSteering();

    //UART_PORT.print("throttle: ");
    //UART_PORT.println(throttle);
    //UART_PORT.print("steering: ");
    //UART_PORT.println(steering);


    // If statement for CASE 1: steering toward the RIGHT
    if (steering > 0 ) {
      deg = mapFloat(steering, 0, maxS, 1, -1);
      desiredVelocityRight = map(throttle * deg, minS, maxS, -255, 255);
      desiredVelocityLeft = map(throttle, minS, maxS,  -255, 255);
      rotation = "CW";
    }


    // If statement for CASE 2: steering toward the LEFT or not steering
    if (steering <= 0 ) {
      deg = mapFloat(steering, minS, 0, -1, 1);
      desiredVelocityRight = map(throttle, minS, maxS, -255, 255);
      desiredVelocityLeft = map(throttle * deg, minS, maxS, -255, 255);
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

    RF.setVelocityNoPID(rightMotorDirection, abs(desiredVelocityRight));
    RM.setVelocityNoPID(rightMotorDirection, abs(desiredVelocityRight));
    RB.setVelocityNoPID(rightMotorDirection, abs(desiredVelocityRight));
    UART_PORT.print("rightMotorDirection: ");
    UART_PORT.println(abs(rightMotorDirection));
    UART_PORT.print("desiredVelocityRight: ");
    UART_PORT.println(abs(desiredVelocityRight));

    LF.setVelocityNoPID(leftMotorDirection, abs(desiredVelocityLeft));
    LM.setVelocityNoPID(leftMotorDirection, abs(desiredVelocityLeft));
    LB.setVelocityNoPID(leftMotorDirection, abs(desiredVelocityLeft));
    UART_PORT.print("leftMotorDirection: ");
    UART_PORT.println(abs(leftMotorDirection));
    UART_PORT.print("desiredVelocityLeft: ");
    UART_PORT.println(abs(desiredVelocityLeft));


    RF.calcCurrentVelocity();
    RM.calcCurrentVelocity();
    RB.calcCurrentVelocity();
    LF.calcCurrentVelocity();
    LM.calcCurrentVelocity();
    LB.calcCurrentVelocity();

    vr = RF.getDirection() * RF.getCurrentVelocity() + RM.getDirection() * RM.getCurrentVelocity() + RB.getDirection() * RB.getCurrentVelocity();
    vl = LF.getDirection() * LF.getCurrentVelocity() + LM.getDirection() * LM.getCurrentVelocity() + LB.getDirection() * LB.getCurrentVelocity();

    vx = (vr + vl) / 6;
    vth = (vl - vr) / d;

    //Serial.print(desiredVelocityLeft);  Serial.print("_________"); Serial.println(desiredVelocityRight);
    //Serial.print(rightMotorDirection); Serial.print(rotation); Serial.print("_____");Serial.print(deg);Serial.print("_____"); Serial.println(steering);
    //Serial.print("Velocity:"); Serial.println(v);


    //ser_flush();
    prevRead = millis();
  }

}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void dcInterrupt(void) {
  ;

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
}

void initEncoders(void) {
  RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
  pinMode(RF.encoderPinB, INPUT_PULLUP);
  pinMode(RF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, RISING);
  //    attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, RISING);
  RF.pidController.setGainConstants(1.0, 0.0, 0.0);

  RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
  pinMode(RM.encoderPinB, INPUT_PULLUP);
  pinMode(RM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, RISING);
  //    attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, RISING);
  RM.pidController.setGainConstants(1.0, 0.0, 0.0);

  RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
  pinMode(RB.encoderPinB, INPUT_PULLUP);
  pinMode(RB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, RISING);
  //  attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, RISING);
  RB.pidController.setGainConstants(1.0, 0.0, 0.0);

  LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
  pinMode(LF.encoderPinB, INPUT_PULLUP);
  pinMode(LF.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, RISING);
  //    attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, RISING);
  LF.pidController.setGainConstants(1.0, 0.0, 0.0);

  LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
  pinMode(LM.encoderPinB, INPUT_PULLUP);
  pinMode(LM.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, RISING);
  //   attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, RISING);
  LM.pidController.setGainConstants(1.0, 0.0, 0.0);

  LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
  pinMode(LB.encoderPinB, INPUT_PULLUP);
  pinMode(LB.encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, RISING);
  //    attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, RISING);
  LB.pidController.setGainConstants(1.0, 0.0, 0.0);
}

void initPids(void) {
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
}

#ifdef TEST

void test_encoder_interrupt(void) {
  test.encoderCount++;
  // dt = micros() - prevTime;

  //  prevTime = micros();

}

#else

void rf_encoder_interrupt(void) {

  RF.dt = micros() - prevTime;
  RF.prevTime = micros();
}

void rm_encoder_interrupt(void) {
  RM.dt = micros() - prevTime;
  RM.prevTime = micros();
}

void rb_encoder_interrupt(void) {
  RB.dt = micros() - prevTime;
  RB.prevTime = micros();
}


void lf_encoder_interrupt(void) {
  LF.dt = micros() - prevTime;
  LF.prevTime = micros();
}


void lm_encoder_interrupt(void) {
  LM.dt = micros() - prevTime;
  LM.prevTime = micros();
}


void lb_encoder_interrupt(void) {
  LB.dt = micros() - prevTime;
  LB.prevTime = micros();
}

#endif
void encoder_ISR(void) {

  dt = micros() - prevTime;
  prevTime = micros();

  interruptCount++;
}
