#include <ArduinoBlue.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <elapsedMillis.h>
#include "DcMotor.h"
 

#define PULSES_PER_REV      7
#define GEAR_RATIO         71.16
//#define GEAR_RATIO         188.61
 
//
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
#define TEST_DIR   5
#define TEST_PWM   9
#define TEST_EA   2
#define TEST_EB   3

#define RF_DIR   11
#define RM_DIR   12
#define RB_DIR   24
 
#define LF_DIR   5
#define LM_DIR   6
#define LB_DIR   7
 
#define RF_PWM   2
#define RF_PWM   3
#define RB_PWM   4
 
#define LF_PWM   5
#define LM_PWM   6
#define LB_PWM   7

#define RF_EA    25
#define RF_EB    26

#define RM_EA    27
#define RM_EB    28

#define RB_EA    29
#define RB_EB    30

#define LF_EA    31
#define LF_EB    32

#define LM_EA    33
#define LM_EB    34

#define LB_EA    35
#define LB_EB    36

#define DC_SIGNAL_MIN 0
#define DC_SIGNAL_MAX 255


// This portion shall be removed once ROSyfied
int steeringShiftValue = 49;
int throttleShiftValue = 49;
 
float minThrottleController = 0 - throttleShiftValue;
float maxThrottleController = 0 + throttleShiftValue;
float minSteeringController = 0 - steeringShiftValue;
float maxSteeringController = 0 + steeringShiftValue;

int throttle, steering;
float deg = 0;


unsigned long startTime = millis();
unsigned long currTime = millis();
volatile unsigned long interruptCount = 0;
volatile unsigned long prevTime = micros();
volatile unsigned long dt;
float degAngularResolution = 1 / (float)(GEAR_RATIO*PULSES_PER_REV);
 
float velocity_Right = 0;
float velocity_Left = 0;
 
// Right side servos
unsigned int prevRead = millis();
 
int prevThrottle = 0;
int prevSteering = 0;
 
float vy = 0;
float vx, vth;
 
String rotation;
 
SoftwareSerial bluetooth(7, 8);
ArduinoBlue phone(bluetooth);
 
 
void velocityHandler(float throttle);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
 
void encoder_ISR(void);

void test_encoder_interrupt(void);

void rf_encoder_interrupt(void);
void rm_encoder_interrupt(void);
void rb_encoder_interrupt(void);
void lf_encoder_interrupt(void);
void lm_encoder_interrupt(void);
void lb_encoder_interrupt(void);

IntervalTimer dcTimer;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  bluetooth.begin(9600);
 
  pinMode(DC_PIN_FRONT_RIGHT, OUTPUT);
   // Write velocities for the Wheels on the RIGHT side
  analogWrite(DC_PIN_FRONT_RIGHT, 0);
 
 
  pinMode(DIR_PIN_FRONT_RIGHT, OUTPUT); delay(10);
 
  // this sets up the initial rotation direction to counter clockwise, so High = CCW
  digitalWrite(DIR_PIN_FRONT_RIGHT, HIGH);
 
  pinMode(M2_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_B), encoder_ISR, FALLING); delay(10);



#ifdef TEST_DIR

  DcMotor TEST(TEST_DIR, TEST_PWM, GEAR_RATIO);
  TEST.attachEncoder(RF_EA, RF_EB);
  attachInterrupt(digitalPinToInterrupt(TEST.encoderPinA), test_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TEST.encoderPinB), test_encoder_interrupt, CHANGE);
  TEST.pidController.setGainConstants(1.0, 0.0, 0.0);

  TEST.pidController.setJointVelocityTolerance(2.0 * TEST.gearRatioReciprocal);
#else
  DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO);
  DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO);
  DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO);
  
  DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO);
  DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO);
  DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO);

  RF.attachEncoder(RF_EA, RF_EB);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, CHANGE);
  RF.pidController.setGainConstants(1.0, 0.0, 0.0);

  RM.attachEncoder(RM_EA, RM_EB);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, CHANGE);
  RM.pidController.setGainConstants(1.0, 0.0, 0.0);

  RB.attachEncoder(RB_EA, RB_EB);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, CHANGE);
  RB.pidController.setGainConstants(1.0, 0.0, 0.0);

  LF.attachEncoder(LF_EA, LF_EB);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, CHANGE);
  LF.pidController.setGainConstants(1.0, 0.0, 0.0);

  LM.attachEncoder(LM_EA, LM_EB);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, CHANGE);
  LM.pidController.setGainConstants(1.0, 0.0, 0.0);

  LB.attachEncoder(LB_EA, LB_EB);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, CHANGE);
  LB.pidController.setGainConstants(1.0, 0.0, 0.0);

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
  
#endif

dcTimer.begin(dcInterrupt, 20000);

  Serial.println("setup complete");
}
 
void loop() {
  if (millis() - prevRead > 200) {
    // Lead Velocity Value from bluetooth controller. Values range from 0 to 99 for this specific controller
 
        if (Serial.available() > 0) {
                // read the incoming byte:
                velocityRight = Serial.parseFloat() ;
                if (velocityRight = -2 ){
                  digitalWrite(TEST_DIR, LOW);
 
                }
                else if (velocityRight = -1 ){
                   digitalWrite(TEST_DIR, HIGH);
                  }
                else {
                // say what you got:
//                int desiredSpeed = velocityRight.toInt();
//                float rpm = degAngularResolution * 1000 * 1000 * 60 / (float)(dt);
//                int currentSpeed = rpm * blablabla; // convert speed to duty cycle for pwm
//                float output = motorFrontRight.updatePid(currentSpeed, desiredSpeed);
//                dutyCycle = output*255/100;
//                analogWrite(DC_PIN_FRONT_RIGHT, dutyCycle);
 
 
                analogWrite(TEST_PWM, velocityRight);
                //analogWrite(DC_PIN_FRONT_RIGHT, velocityRight.toInt());
                Serial.print("Throttle: "); Serial.print(velocityRight);
               
               Serial.print("\tCount: "); Serial.println(interruptCount);
                float rpm = degAngularResolution * 1000 * 1000 * 60 / (float)(dt);
 
                Serial.print("\tRPM: "); Serial.println(rpm,5);
               Serial.print("\tPrev: "); Serial.println(prevTime);
               Serial.print("\tDt: "); Serial.println(dt);
                }
        }
        else {
          //   Lead Velocity Value from bluetooth controller. Values range from 0 to 99 for this specific controller
            throttle = phone.getThrottle();
            // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
            steering = phone.getSteering();
            // Function that updates velocity values based on steering angle.
            throttle -= throttleShiftValue;
            steering -= steeringShiftValue;
            
         // If statement for CASE 1: steering toward the RIGHT
            if (steering < 0 ) {
              deg = mapFloat(steering, minSteeringController, 0, -1, 1);
              desiredVelocityRight = mapFloat(throttle * deg, minThrottleController, maxThrottleController, 0, 75);
              desiredVelocityLeft = mapFloat((-1) * throttle, minThrottleController, maxThrottleController,  0, 75);
           
              rotation = "CW";
          //    if moving forwad we have CCW rotation so dir is HIGH
             
            }
         
          // If statement for CASE 2: steering toward the LEFT
            if (steering >= 0 ) {
              deg = mapFloat(steering, 0, maxSteeringController, 1, -1);
              desiredVelocityRight = mapFloat(throttle, minThrottleController, maxThrottleController, 0, 75);
              desiredVelocityLeft = mapFloat((-1) * throttle * deg, minThrottleController, maxThrottleController, 0, 75);
              rotation = "CCW";
           
            }
           
          if ( velocityRight >= 0 ){
                 RF.setDesiredDirection(CCW);
             }
             else {
                 RF.setDesiredDirection(CW);
           
             }
              if ( velocityLeft >= 0 ){
                 LF.setDesiredDirection(CW);
             }
             else {
                 LF.setDesiredDirection(CCW);
           
             }
            
//    
          }
 
 
 
 
    prevRead = millis();
  }
 
}
 
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 
void velocityHandler(float throttle, float steering) {
  
 
 
 
   
  // Print Velocity Values
  if (prevThrottle != throttle || prevSteering != steering) {
    float rpm = degAngularResolution * 1000 * 1000 * 60 / (float)(dt);
   
    Serial.print("\tThrottle: "); Serial.print(throttle);
    Serial.print("\tSteering: "); Serial.println(steering);
    Serial.print("\tRotation: "); Serial.println(rotation);
    Serial.print("\tRight Side: "); Serial.println(velocityRight);
    Serial.print("\tLeft Side: "); Serial.println(velocityLeft);
   
               
     Serial.print("\tCount: "); Serial.println(interruptCount);
 
      Serial.print("\tRPM: "); Serial.println(rpm,5);
     Serial.print("\tPrev: "); Serial.println(prevTime);
     Serial.print("\tDt: "); Serial.println(dt);
     
    prevThrottle = throttle;
    prevSteering = steering;
  }
 
 
  // Write velocities for the Wheels on the RIGHT side
  analogWrite(DC_PIN_FRONT_RIGHT, fabs(velocityRight));
  analogWrite(DC_PIN_MIDDLE_RIGHT, fabs(velocityRight));
  analogWrite(DC_PIN_BACK_RIGHT, fabs(velocityRight));
 
  // Write velocities for the Wheels on the Left side
  analogWrite(DC_PIN_FRONT_LEFT, fabs(velocityLeft));
  analogWrite(DC_PIN_MIDDLE_LEFT, fabs(velocityLeft));
  analogWrite(DC_PIN_BACK_LEFT, fabs(velocityLeft));
 
 
}
 
void encoder_ISR(void) {
    dt = micros() - prevTime;
    prevTime = micros();
 
  interruptCount++;
}
