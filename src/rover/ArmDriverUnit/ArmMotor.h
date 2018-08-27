#ifndef ARMMOTOR_H
#define ARMMOTOR_H

#include "PinSetup.h"
#include "AbtinEncoder.h"

//#include <AccelStepper.h>
//#include <MultiStepper.h>

/*
   m1 base stepper (rotation)
   m2 shoulder dc (flexion)
   m3 elbow stepper (flexion)
   m4 wrist stepper (flexion
   m5 wrist servo (rotation)
   m6 end effector servo (pinching)
*/

// 3 pwm pins
// 4-6 limit switch interrupt pins
// 8-14 encoder interrupt pins
// 3 direction pins, 3 step pins

/*
  #define M1_STEP_HIGH        PORTD |=  0b10000000;
  #define M1_STEP_LOW         PORTD &= ~0b10000000;

  #define M2_STEP_HIGH        PORTD |=  0b00100000;
  #define M2_STEP_LOW         PORTD &= ~0b00100000;

  #define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
  #define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);
*/

enum motor_code {MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6};
enum motor_type {DC_MOTOR, STEPPER_MOTOR, SERVO_MOTOR};
enum motor_direction {CLOCKWISE, COUNTER_CLOCKWISE};
enum motor_speed {SPEED0, SPEED1, SPEED2, SPEED3};

#define MAX_SPEED 3
#define DEFAULT_SPEED 0

#define PWM_STOP 127
// speed 0(+/-32)
#define PWM_CW0 160
#define PWM_CCW0 96
// speed 1 (+/-64)
#define PWM_CW1 192
#define PWM_CCW1 64
// speed 2 (+/-96)
#define PWM_CW2 224
#define PWM_CCW2 32
// speed 3 (+/-127)
#define PWM_CW3 255
#define PWM_CCW3 1

#define STEP_INTERVAL0 10
#define STEP_INTERVAL1 25
#define STEP_INTERVAL2 50
#define STEP_INTERVAL3 100

#define MAX_BUDGE_TIME 3000
#define MIN_BUDGE_TIME 100
#define DEFAULT_BUDGE_TIME 500

/*
  unsigned int c0 = 1600;  // was 2000 * sqrt( 2 * angle / accel )

  struct stepperInfo {
  void (*dirFunc)(int);
  void (*stepFunc)();
  volatile float dir = 0;
  volatile float currentAngle = 0;
  volatile float desiredAngle = 0;
  volatile bool movementDone = true;
  };

  struct dcInfo {
  volatile int dir = 127; // 0 to 255 where 127 is the midpoint and stops the motor
  volatile float currentAngle = 0;
  volatile float desiredAngle = 0;
  volatile bool dcMovementDone = false;
  volatile long encCount = 0;
  };

  struct servoInfo {
  volatile int dir = 127; // 0 to 255 where 127 is the midpoint and stops the motor
  volatile float currentAngle = 0;
  volatile float desiredAngle = 0;
  volatile bool dcMovementDone = false;
  volatile long encCount = 0;
  };

  volatile stepperInfo steppers[NUM_STEPPERS];
  char serialBuffer[BUFFER_SIZE];
  volatile dcInfo dcMotor;
  bool led_status = true;
  int startTime = 0;

  void M1Step() {
  M1_STEP_HIGH
  M1_STEP_LOW
  }
  void M1Dir(int d) {
  digitalWrite(M1_DIR_PIN, d);
  }

  void M2Step() {
  M2_STEP_HIGH
  M2_STEP_LOW
  }
  void M2Dir(int d) {
  digitalWrite(M2_DIR_PIN, d);
  }
*/
// todo: finish everything, make sure to use the right pins for the encoders
class ArmMotor {
  public:
    //volatile int encoderCount;
    elapsedMillis sinceStart = 0; // for time of motor budging
    elapsedMillis sinceStep = 0; // for time between stepper steps

    ArmMotor(int code);

    void setMotorSpeed();
    float getCurrentAngle();

    void budge(int budgeDir, int budgeSpeed, unsigned int budgeTime);
    void setDesiredAngle(float angle);
    void update();
  private:
    int motorCode, motorType; // code determines motor 1-6, type determines dc vs stepper vs servo
    int pwmPin, enablePin, dirPin, stepPin; // first is for dc/servo, next 3 are for steppers
    float currentAngle;
    float desiredAngle;
    bool isMovementDone;

};

ArmMotor::ArmMotor(int code): motorCode(code) {
  // this code only runs at boot so order of switch statement not important
  switch (motorCode) {
    case MOTOR1:
      motorType = STEPPER_MOTOR;
      enablePin = M1_ENABLE_PIN;
      dirPin = M1_DIR_PIN;
      stepPin = M1_STEP_PIN;
      attachInterrupt(M1_ENCODER_A, m1_encoder_interrupt, CHANGE);
      attachInterrupt(M1_ENCODER_B, m1_encoder_interrupt, CHANGE);
      break;
    case MOTOR2:
      motorType = DC_MOTOR;
      pwmPin = M2_PWM_PIN;
      attachInterrupt(M2_ENCODER_A, m2_encoder_interrupt, CHANGE);
      attachInterrupt(M2_ENCODER_B, m2_encoder_interrupt, CHANGE);
      break;
    case MOTOR3:
      motorType = STEPPER_MOTOR;
      enablePin = M3_ENABLE_PIN;
      dirPin = M3_DIR_PIN;
      stepPin = M3_STEP_PIN;
      attachInterrupt(M3_ENCODER_A, m3_encoder_interrupt, CHANGE);
      attachInterrupt(M3_ENCODER_B, m3_encoder_interrupt, CHANGE);
      break;
    case MOTOR4:
      motorType = STEPPER_MOTOR;
      enablePin = M4_ENABLE_PIN;
      dirPin = M4_DIR_PIN;
      stepPin = M4_STEP_PIN;
      attachInterrupt(M4_ENCODER_A, m4_encoder_interrupt, CHANGE);
      attachInterrupt(M4_ENCODER_B, m4_encoder_interrupt, CHANGE);
      break;
    case MOTOR5:
      motorType = SERVO_MOTOR;
      pwmPin = M5_PWM_PIN;
      break;
    case MOTOR6:
      motorType = SERVO_MOTOR;
      pwmPin = M6_PWM_PIN;
      break;
  }
}

/*
   there could be background interrupt routines for all the motor types that wait for instructions...
   so far we want the automatic control to be done through interrupts, and the thing that tells the
   ISR how to move is the pid controller. so for manual control, i need another set of functions that
   tell the ISR how to move. but for now I can just make simple code that does the thing, I suppose.
*/

void ArmMotor::budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME) {
  // arranged in order of which motor is predicted to be controlled the most
  int cwSpeed, ccwSpeed, stepInterval;
  if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    switch (motorType) {
      case STEPPER_MOTOR:
        digitalWriteFast(enablePin, HIGH);
        switch (budgeDir) {
          case CLOCKWISE:
            digitalWriteFast(dirPin, HIGH);
            break;
          case COUNTER_CLOCKWISE:
            digitalWriteFast(dirPin, LOW);
            break;
        }
        switch (budgeSpeed) {
          case 0:
            stepInterval = STEP_INTERVAL0;
            break;
          case 1:
            stepInterval = STEP_INTERVAL1;
            break;
          case 2:
            stepInterval = STEP_INTERVAL2;
            break;
          case 3:
            stepInterval = STEP_INTERVAL3;
            break;
        }
        sinceStart = 0;
        while (sinceStart < budgeTime) {
          // implement speed control
          digitalWriteFast(stepPin, HIGH);
          digitalWriteFast(stepPin, LOW);
          sinceStep = 0;
		  while (sinceStep < stepInterval) ;
        }
        digitalWriteFast(enablePin, LOW);
        break;
      case DC_MOTOR: // code slightly different from pwm now because of pinmodes in servo
        switch (budgeSpeed) {
          case 0:
            cwSpeed = PWM_CW0; ccwSpeed = PWM_CCW0;
            break;
          case 1:
            cwSpeed = PWM_CW1; ccwSpeed = PWM_CCW1;
            break;
          case 2:
            cwSpeed = PWM_CW2; ccwSpeed = PWM_CCW2;
            break;
          case 3:
            cwSpeed = PWM_CW3; ccwSpeed = PWM_CCW3;
            break;
        }
        Serial.println("starting");
        sinceStart = 0;
        if (budgeDir == CLOCKWISE) {
          analogWrite(pwmPin, cwSpeed);
          while (sinceStart < budgeTime) ;
          analogWrite(pwmPin, PWM_STOP);
        }
        if (budgeDir == COUNTER_CLOCKWISE) {
          analogWrite(pwmPin, ccwSpeed);
          while (sinceStart < budgeTime) ;
          analogWrite(pwmPin, PWM_STOP);
        }
        Serial.println("stopping");
        break;
      case SERVO_MOTOR:
        switch (budgeSpeed) {
          case 0:
            cwSpeed = PWM_CW0; ccwSpeed = PWM_CCW0;
            break;
          case 1:
            cwSpeed = PWM_CW1; ccwSpeed = PWM_CCW1;
            break;
          case 2:
            cwSpeed = PWM_CW2; ccwSpeed = PWM_CCW2;
            break;
          case 3:
            cwSpeed = PWM_CW3; ccwSpeed = PWM_CCW3;
            break;
        }
        Serial.println("starting");
		pinMode(pwmPin, OUTPUT);
        sinceStart = 0;
        if (budgeDir == CLOCKWISE) {
          analogWrite(pwmPin, cwSpeed);
          while (sinceStart < budgeTime) ;
          analogWrite(pwmPin, PWM_STOP);
        }
        if (budgeDir == COUNTER_CLOCKWISE) {
          analogWrite(pwmPin, ccwSpeed);
          while (sinceStart < budgeTime) ;
          analogWrite(pwmPin, PWM_STOP);
        }
		pinMode(pwmPin, INPUT);
        Serial.println("stopping");
        break;
    }
  }
}

void ArmMotor::setDesiredAngle(float angle) {
  // arranged in order of which motor is predicted to be controlled the most
  switch (motorType) {
    case STEPPER_MOTOR:
      //stepper pid
      break;
    case DC_MOTOR:
      //dc pid
      break;
    case SERVO_MOTOR:
      //servo pid? idk if we'll use this
      break;
  }
}
#endif
