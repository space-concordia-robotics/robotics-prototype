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
/*
  3 pwm pins
  4-6 limit switch interrupt pins
  8-14 encoder interrupt pins
  3 direction pins, 3 step pins
*/
/*
  #define M1_STEP_HIGH        PORTD |=  0b10000000;
  #define M1_STEP_LOW         PORTD &= ~0b10000000;

  #define M2_STEP_HIGH        PORTD |=  0b00100000;
  #define M2_STEP_LOW         PORTD &= ~0b00100000;

  #define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
  #define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);
*/

enum motor_code {MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6}; // defines 6 motors
enum motor_type {DC_MOTOR, STEPPER_MOTOR, SERVO_MOTOR}; // defines 3 motor types
enum motor_direction {CLOCKWISE, COUNTER_CLOCKWISE}; // defines motor directions
enum motor_speed {SPEED0, SPEED1, SPEED2, SPEED3}; // defines motor speed

#define MAX_SPEED 3 // 4 speed options
#define DEFAULT_SPEED 0 // default speed is slowest
#define MAX_COUNTS 5 // max number of times a motor can turn in a specific direction

// pwm speed control
#define PWM_STOP 127 // may not be used but motor is supposed to stop at 50% duty cycle (127/255)
// speed 0, slowest (+/-32)
#define PWM_CW0 160
#define PWM_CCW0 96
// speed 1 (+/-64)
#define PWM_CW1 192
#define PWM_CCW1 64
// speed 2 (+/-96)
#define PWM_CW2 224
#define PWM_CCW2 32
// speed 3, fastest (+/-127)
#define PWM_CW3 255
#define PWM_CCW3 1

// time interval between stepper steps
#define STEP_INTERVAL0 10
#define STEP_INTERVAL1 25
#define STEP_INTERVAL2 50
#define STEP_INTERVAL3 100

// limits in ms for amount of time the motor can budge
#define MAX_BUDGE_TIME 3000
#define MIN_BUDGE_TIME 100
#define DEFAULT_BUDGE_TIME 500

class ArmMotor {
  public:
    //volatile int encoderCount;
    
    //void setMotorSpeed();
    //float getCurrentAngle();

    virtual void budge(int budgeDir, int budgeSpeed, unsigned int budgeTime); // budges motor for short period of time
    //void setDesiredAngle(float angle);
    //void update();
  private:
    //int motorCode = 0; // code determines motor 1-6, probably not needed but could be important
    elapsedMillis sinceStart = 0; // for time of motor budging
	int cwSpeed, ccwSpeed; unsigned int stepInterval;

    int rightCount, leftCount = 0; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    //float currentAngle, desiredAngle; // for angle control
    //bool isMovementDone;
};

/* placed cwspeed, ccwspeed, stepinterval inside armmotor instead of budge() */

class StepperMotor: public ArmMotor {
	public:
		StepperMotor();
		void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
	private:
		int enablePin, dirPin, stepPin;
		elapsedMillis sinceStep = 0; // for time between steps
};

StepperMotor::StepperMotor(int code){
	switch(code){
		case MOTOR1:
		  enablePin = M1_ENABLE_PIN; // gives power to stepper
		  dirPin = M1_DIR_PIN; // sets stepper rotation direction
		  stepPin = M1_STEP_PIN; // rising edge causes step
		  attachInterrupt(M1_ENCODER_A, m1_encoder_interrupt, CHANGE);
		  attachInterrupt(M1_ENCODER_B, m1_encoder_interrupt, CHANGE);
		  break;
		case MOTOR3:
		  enablePin = M3_ENABLE_PIN;
		  dirPin = M3_DIR_PIN;
		  stepPin = M3_STEP_PIN;
		  attachInterrupt(M3_ENCODER_A, m3_encoder_interrupt, CHANGE);
		  attachInterrupt(M3_ENCODER_B, m3_encoder_interrupt, CHANGE);
		  break;
		case MOTOR4:
		  enablePin = M4_ENABLE_PIN;
		  dirPin = M4_DIR_PIN;
		  stepPin = M4_STEP_PIN;
		  attachInterrupt(M4_ENCODER_A, m4_encoder_interrupt, CHANGE);
		  attachInterrupt(M4_ENCODER_B, m4_encoder_interrupt, CHANGE);
		  break;
	}
}

StepperMotor::budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME){
	if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
		if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
		  canTurnRight = true;
		  rightCount++; leftCount--;
		}
		if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
		  canTurnLeft = true;
		  leftCount++; rightCount--;
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
		if (budgeDir == CLOCKWISE && canTurnRight) {
		  digitalWriteFast(dirPin, HIGH);
		  digitalWriteFast(enablePin, HIGH);
		  sinceStart = 0;
		  while (sinceStart < budgeTime) {
			// motor driver is fast enough to recognize this quickly rising and falling edge
			digitalWriteFast(stepPin, HIGH);
			digitalWriteFast(stepPin, LOW);
			sinceStep = 0;
			while (sinceStep < stepInterval) ; // wait until it's time to step again
		  }
		}
		if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
		  digitalWriteFast(dirPin, LOW);
		  digitalWriteFast(enablePin, HIGH);
		  sinceStart = 0;
		  while (sinceStart < budgeTime) {
			// motor driver is fast enough to recognize this quickly rising and falling edge
			digitalWriteFast(stepPin, HIGH);
			digitalWriteFast(stepPin, LOW);
			sinceStep = 0;
			while (sinceStep < stepInterval) ; // wait until it's time to step again
		  }
		}
	digitalWriteFast(enablePin, LOW); // be sure to disconnect power to stepper so it doesn't get hot / drain power
	}
}

class DCMotor: public ArmMotor {
	public:
		DCMotor();
		void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
	private:
		int pwmPin;
};

DCMotor::DCMotor(int code){
    switch(code){
		case MOTOR2:
		  pwmPin = M2_PWM_PIN;
		  attachInterrupt(M2_ENCODER_A, m2_encoder_interrupt, CHANGE);
		  attachInterrupt(M2_ENCODER_B, m2_encoder_interrupt, CHANGE);
		  break;
}

DCMotor::budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME){
		if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
		if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
		  canTurnRight = true;
		  rightCount++; leftCount--;
		}
		if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
		  canTurnLeft = true;
		  leftCount++; rightCount--;
		}
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
        if (budgeDir == CLOCKWISE && canTurnRight) {
          analogWrite(pwmPin, cwSpeed);
          while (sinceStart < budgeTime) ; // wait
          analogWrite(pwmPin, PWM_STOP); // sets duty cycle to 50% which corresponds to 0 speed
        }
        if (budgeDir == COUNTER_CLOCKWISE && canTurnLeft) {
          analogWrite(pwmPin, ccwSpeed);
          while (sinceStart < budgeTime) ; // wait
          analogWrite(pwmPin, PWM_STOP); // sets duty cycle to 50% which corresponds to 0 speed
        }
        Serial.println("stopping");
        break;
		}
}

class ServoMotor: public ArmMotor {
	public:
		ServoMotor();
		void budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME);
	private:
		int pwmPin;
};

ServoMotor::ServoMotor(){
	case MOTOR5:
      pwmPin = M5_PWM_PIN;
      break;
    case MOTOR6:
      pwmPin = M6_PWM_PIN;
      break;
};

/*
   there could be background interrupt routines for all the motor types that wait for instructions...
   so far we want the automatic control to be done through interrupts, and the thing that tells the
   ISR how to move is the pid controller. so for manual control, i need another set of functions that
   tell the ISR how to move. but for now I can just make simple code that does the thing, I suppose.
*/

void ServoMotor::budge(int budgeDir = CLOCKWISE, int budgeSpeed = DEFAULT_SPEED, unsigned int budgeTime = DEFAULT_BUDGE_TIME) {
  // arranged in order of which motor is predicted to be controlled the most
  		if (budgeDir <= 1 && budgeSpeed <= MAX_SPEED && budgeTime <= MAX_BUDGE_TIME && budgeTime >= MIN_BUDGE_TIME) {
    // following if statements ensure motor only moves if within count limit, updates current count
		if (budgeDir == CLOCKWISE && rightCount < MAX_COUNTS) {
		  canTurnRight = true;
		  rightCount++; leftCount--;
		}
		if (budgeDir == COUNTER_CLOCKWISE && leftCount < MAX_COUNTS) {
		  canTurnLeft = true;
		  leftCount++; rightCount--;
		}
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
        if (budgeDir == CLOCKWISE && canTurnRight) {
          analogWrite(pwmPin, cwSpeed);
          while (sinceStart < budgeTime) ; // wait
          analogWrite(pwmPin, PWM_STOP); // sets duty cycle to 50% which corresponds to 0 speed
        }
        if (budgeDir == CLOCKWISE && canTurnRight) {
          analogWrite(pwmPin, ccwSpeed);
          while (sinceStart < budgeTime) ; // wait
          analogWrite(pwmPin, PWM_STOP); // sets duty cycle to 50% which corresponds to 0 speed
        }
        pinMode(pwmPin, INPUT); // sets pin to floating input. This is necessary to cut power to servo because otherwise it jitters, gets hot, drains power
        Serial.println("stopping");
        break;
    }
  }
  canTurnRight = false; canTurnLeft = false; // reset bools

}

/*
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
*/

#endif
