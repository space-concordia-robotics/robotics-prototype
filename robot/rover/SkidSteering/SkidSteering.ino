#include <ArduinoBlue.h>
#include <SoftwareSerial.h>
//#include <Servo.h>
#include <elapsedMillis.h>
#include "DcMotor.h"

#define mode_0 //sabertooth drivers for initiation testing
//#define mode_1 //Cytron drivers

#define TESTs

#define NO_PID // PidController not active

#define PULSES_PER_REV      7
//#define GEAR_RATIO         71.16
#define maxVelocity         75
#define minVelocity         -75
#define GEAR_RATIO         188.61
float prevCurrent = 0;
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
#ifdef TEST
#define TEST_DIR   5
#define TEST_PWM   9
#define TEST_EA   2
#define TEST_EB   3

#endif

#define RF_DIR   11
#define RM_DIR   12
#define RB_DIR   24

#define LF_DIR   8
#define LM_DIR   9
#define LB_DIR   10

#define RF_PWM   2
#define RM_PWM   3
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
int steeringShiftValue = 49;
int throttleShiftValue = 49;

float minThrottleController = 0 - throttleShiftValue;
float maxThrottleController = 0 + throttleShiftValue;
float minSteeringController = 0 - steeringShiftValue;
float maxSteeringController = 0 + steeringShiftValue;

int throttle, steering;

float deg = 0;
float rpm =0;

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

#ifdef mode_0
#define MAX_PWM 255
#define MIN_PWM 0
#endif
String rotation;

SoftwareSerial bluetooth(7, 8);
ArduinoBlue phone(bluetooth);


void velocityHandler(float throttle);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

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

void setup() {
    // initialize serial communications at 9600 bps:
    Serial.begin(9600);
    bluetooth.begin(9600);

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
    DcMotor RF(RF_DIR, RF_PWM, GEAR_RATIO);
    DcMotor RM(RM_DIR, RM_PWM, GEAR_RATIO);
    DcMotor RB(RB_DIR, RB_PWM, GEAR_RATIO);

    DcMotor LF(LF_DIR, LF_PWM, GEAR_RATIO);
    DcMotor LM(LM_DIR, LM_PWM, GEAR_RATIO);
    DcMotor LB(LB_DIR, LB_PWM, GEAR_RATIO);

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

    
    RF.attachEncoder(RF_EA, RF_EB, PULSES_PER_REV);
    pinMode(RF.encoderPinB, INPUT_PULLUP);
    pinMode(RF.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RF.encoderPinA), rf_encoder_interrupt, RISING);
    // attachInterrupt(digitalPinToInterrupt(RF.encoderPinB), rf_encoder_interrupt, RISING);
    RF.pidController.setGainConstants(1.0, 0.0, 0.0);



    RM.attachEncoder(RM_EA, RM_EB, PULSES_PER_REV);
    pinMode(RM.encoderPinB, INPUT_PULLUP);
    pinMode(RM.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RM.encoderPinA), rm_encoder_interrupt, RISING);
    // attachInterrupt(digitalPinToInterrupt(RM.encoderPinB), rm_encoder_interrupt, RISING);
    RM.pidController.setGainConstants(1.0, 0.0, 0.0);



    RB.attachEncoder(RB_EA, RB_EB, PULSES_PER_REV);
    pinMode(RB.encoderPinB, INPUT_PULLUP);
    pinMode(RB.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RB.encoderPinA), rb_encoder_interrupt, RISING);
    // attachInterrupt(digitalPinToInterrupt(RB.encoderPinB), rb_encoder_interrupt, RISING);
    RB.pidController.setGainConstants(1.0, 0.0, 0.0);




    LF.attachEncoder(LF_EA, LF_EB, PULSES_PER_REV);
    pinMode(LF.encoderPinB, INPUT_PULLUP);    
    pinMode(LF.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LF.encoderPinA), lf_encoder_interrupt, RISING);
    // attachInterrupt(digitalPinToInterrupt(LF.encoderPinB), lf_encoder_interrupt, RISING);
    LF.pidController.setGainConstants(1.0, 0.0, 0.0);




    LM.attachEncoder(LM_EA, LM_EB, PULSES_PER_REV);
    pinMode(LM.encoderPinB, INPUT_PULLUP);
    pinMode(LM.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LM.encoderPinA), lm_encoder_interrupt, RISING);
    // attachInterrupt(digitalPinToInterrupt(LM.encoderPinB), lm_encoder_interrupt, RISING);
    LM.pidController.setGainConstants(1.0, 0.0, 0.0);



    LB.attachEncoder(LB_EA, LB_EB, PULSES_PER_REV);
    pinMode(LB.encoderPinB, INPUT_PULLUP);    
    pinMode(LB.encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LB.encoderPinA), lb_encoder_interrupt, RISING);
    // attachInterrupt(digitalPinToInterrupt(LB.encoderPinB), lb_encoder_interrupt, RISING);
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

}

void loop() {
    if (millis() - prevRead > 1000)
    {
        // Steering Value from bluetooth controller. Values range from 0 to 99 for this specific controller
        throttle = phone.getThrottle();
        steering = phone.getSteering();

        // Function that updates velocity values based on steering angle.

        throttle -= throttleShiftValue;
        steering -= steeringShiftValue;

        #ifdef mode_0


            // If statement for CASE 1: steering toward the RIGHT
            if (steering < 0 ) {
                deg = mapFloat(steering, minSteeringController, 0, -1, 1);
                desiredVelocityRight = map(throttle * deg, minThrottleController, maxThrottleController, MIN_PWM, MAX_PWM);
                desiredVelocityLeft = map((-1) * throttle, minThrottleController, maxThrottleController,  MIN_PWM, MAX_PWM);
            }

            // If statement for CASE 2: steering toward the LEFT
            if (steering >= 0 ) {
                deg = mapFloat(steering, 0, maxSteeringController, 1, -1);
                desiredVelocityRight = map(throttle, minThrottleController, maxThrottleController, MIN_PWM, MAX_PWM);
                desiredVelocityLeft = map((-1) * throttle * deg, minThrottleController, maxThrottleController, MIN_PWM, MAX_PWM);

            }

            #ifdef TEST
                analogWrite(test.getPwmPin(), desiredVelocityRight);

            #else

                analogWrite(RF.getPwmPin(), desiredVelocityRight);
                analogWrite(RM.getPwmPin(), desiredVelocityRight);
                analogWrite(RB.getPwmPin(), desiredVelocityRight);


                analogWrite(LF.getPwmPin(), desiredVelocityLeft);
                analogWrite(LM.getPwmPin(), desiredVelocityLeft);
                analogWrite(LB.getPwmPin(), desiredVelocityRight);

            #endif

        #elif mode_1        //   Code Not complete yet



            // If statement for CASE 1: steering toward the RIGHT
            if (steering < 0 )
            {
                deg = mapFloat(steering, minSteeringController, 0, -1, 1);
                desiredVelocityRight = mapFloat(throttle * deg, 0, maxThrottleController, 0, maxVelocity);
                desiredVelocityLeft = mapFloat((-1) * throttle, 0, maxThrottleController,  0, maxVelocity);
                rf
                rotation = "CW";
                //    if moving forwad we have CCW rotation so dir is HIGH

            }

            // If statement for CASE 2: steering toward the LEFT
            if (steering >= 0 )
            {
                deg = mapFloat(steering, 0, maxSteeringController, 1, -1);
                desiredVelocityRight = mapFloat(throttle, 0, maxThrottleController, 0, maxVelocity);
                desiredVelocityLeft = mapFloat( (-1) * throttle * deg, 0, maxThrottleController, 0, maxVelocity);
                rotation = "CCW";

            }rf



            #ifdef TEST

                if ( desiredVelocityRight >= 0 )
                {
                    test.setDesiredDirection(CCW);
                }
                else
                {
                    test.setDesiredDirection(CW);

                }
                test.setDesiredVelocity(desiredVelocityRight);
                test.calcCurrentVelocity();
                float output = test.pidController.updatePID(test.getDesiredVelocity(), test.getCurrentVelocity());
                #ifdef NO_PID
                    test.setVelocity(test.rotationDirection, test.getDesiredVelocity(), test.getCurrentVelocity());
                #else
                    test.setVelocityNoPID(test.rotationDirection, test.getDesiredVelocity());
                #endif


                rpm = degAngularResolution * 1000 * 1000 * 60 / (float)(dt);
                test.setCurrentVelocity(rpm);
                //      L/F.setDesiredVelocity(desiredVelocityLeft);
                if (prevCurrent != steering ) {
                    Serial.print("\tCount: "); Serial.println(test.encoderCount);

                    Serial.print("\tDesired Velocity"); Serial.println(test.getDesiredVelocity());
                    Serial.print("\tDesired Rotation"); Serial.println(test.rotationDirection);
                    Serial.print("\tPID"); Serial.println(output);
                    Serial.print("\tActual Velocity"); Serial.println(test.getCurrentVelocity());
                    Serial.print("\tPWM Pin "); Serial.println(test.getPwmPin());
                    //       Serial.print("\tdutyCycle "); Serial.print/ln(dutyCycle);
                    prevCurrent = steering;
                }

            #else
                if ( desiredVelocityRight >= 0 )
                {
                    RF.setDesiredDirection(CCW);
                    RM.setDesiredDirection(CCW);
                    RB.setDesiredDirection(CCW);
                }
                else
                {
                    RF.setDesiredDirection(CW);
                    RM.setDesiredDirection(CW);
                    RB.setDesiredDirection(CW);

                }

                 if ( desiredVelocityLeft >= 0 ) {
                     LF.setDesiredDirection(CW);
                     LM.setDesiredDirection(CW);
                     LB.setDesiredDirection(CW);
                 }
                 else
                 {
                     LF.setDesiredDirection(CCW);
                     LM.setDesiredDirection(CCW);
                     LB.setDesiredDirection(CCW);
                   }

                RF.setDesiredVelocity(desiredVelocityRight);
                RM.setDesiredVelocity(desiredVelocityRight);
                RB.setDesiredVelocity(desiredVelocityRight);
                LF.setDesiredVelocity(desiredVelocityLeft);
                LM.setDesiredVelocity(desiredVelocityLeft);
                LB.setDesiredVelocity(desiredVelocityLeft);




                #ifdef NO_PID


                    RF.setVelocityNoPID(RF.rotationDirection, RF.getDesiredVelocity());
                    RM.setVelocityNoPID(RM.rotationDirection, RM.getDesiredVelocity());
                    RB.setVelocityNoPID(RB.rotationDirection, RB.getDesiredVelocity());

                    LF.setVelocityNoPID(LF.rotationDirection, LF.getDesiredVelocity());
                    LM.setVelocityNoPID(LM.rotationDirection, LM.getDesiredVelocity());
                    LB.setVelocityNoPID(LB.rotationDirection, LB.getDesiredVelocity());




                    //      L/F.setDesiredVelocity(desiredVelocityLeft);


                #else // No finished yet


                #endif

            #endif
        #endif
        #ifdef TEST

        #else
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
        #endif

        prevRead = millis();
    }

}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//void velocityHandler(float throttle, float steering) {
//
//
//
//
//
//  // Print Velocity Values
//  if (prevThrottle != throttle || prevSteering != steering) {
//    float rpm = degAngularResolution * 1000 * 1000 * 60 / (float)(dt);
//
//    Serial.print("\tThrottle: "); Serial.print(throttle);
//    Serial.print("\tSteering: "); Serial.println(steering);
//    Serial.print("\tRotation: "); Serial.println(rotation);
//    Serial.print("\tRight Side: "); Serial.println(velocityRight);
//    Serial.print("\tLeft Side: "); Serial.println(velocityLeft);
//
//
//     Serial.print("\tCount: "); Serial.println(interruptCount);
//
//      Serial.print("\tRPM: "); Serial.println(rpm,5);
//     Serial.print("\tPrev: "); Serial.println(prevTime);
//     Serial.print("\tDt: "); Serial.println(dt);
//
//    prevThrottle = throttle;
//    prevSteering = steering;
//  }


// /
///}

void dcInterrupt(void) {


}
#ifdef TEST

void test_encoder_interrupt(void) {
    test.encoderCount++;
    dt = micros() - prevTime;

    prevTime = micros();

}

#else

    void rf_encoder_interrupt(void) {
        RF.encoderCount++;
        dt = micros() - prevTime;
        prevTime = micros();
    }

    void rm_encoder_interrupt(void) {
        RM.encoderCount++;
        dt = micros() - prevTime;
        prevTime = micros();
    }
    void rb_encoder_interrupt(void) {
        RB.encoderCount++;
        dt = micros() - prevTime;
        prevTime = micros();
    }

    void lf_encoder_interrupt(void) {
        LF.encoderCount++;
        dt = micros() - prevTime;
        prevTime = micros();
    }

    void lm_encoder_interrupt(void) {
        LM.encoderCount++;
        dt = micros() - prevTime;
        prevTime = micros();
    }

    void lb_encoder_interrupt(void) {
        LB.encoderCount++;
        dt = micros() - prevTime;
        prevTime = micros();
    }
    #endif
void encoder_ISR(void) {

    dt = micros() - prevTime;
    prevTime = micros();

    interruptCount++;
}
