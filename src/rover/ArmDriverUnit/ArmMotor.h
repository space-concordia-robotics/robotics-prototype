#ifndef ARMMOTOR_H
#define ARMMOTOR_H

#include "PinSetup.h"
#include "AbtinEncoder.h"

/*
 * m1 base stepper (rotation)
 * m2 shoulder dc (flexion)
 * m3 elbow stepper (flexion)
 * m4 wrist stepper (flexion
 * m5 wrist servo (rotation)
 * m6 end effector servo (pinching)
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

enum motor_code {MOTOR1=1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6};
enum motor_type {DC_MOTOR,STEPPER_MOTOR,SERVO_MOTOR};
enum motor_direction {CLOCKWISE,COUNTER_CLOCKWISE};

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
class ArmMotor{
  public:
    //volatile int encoderCount;
    
    ArmMotor(int code);
    
    void setMotorSpeed();
    float getCurrentAngle();
    
    void budge(int rotationDir);
    void setDesiredAngle(float angle);
    void update();
  private:
    int motorCode;
    int motorType;
    float currentAngle;
    float desiredAngle;
    bool isMovementDone;
    
}; //motor1(MOTOR1),motor2(MOTOR2),motor3(MOTOR3),motor4(MOTOR4),motor5(MOTOR5),motor6(MOTOR6);;

//Motor::Motor(int code): motorCode(code){}

//void Motor::init(){
ArmMotor::ArmMotor(int code): motorCode(code){
  // this code only runs at boot so order of switch statement not important
  switch(motorCode){
    case MOTOR1:
      motorType=STEPPER_MOTOR;
      attachInterrupt(M1_ENCODER_A, m1_encoder_interrupt, CHANGE);
      attachInterrupt(M1_ENCODER_B, m1_encoder_interrupt, CHANGE);
      break;
    case MOTOR2:
      motorType=DC_MOTOR;
      attachInterrupt(M2_ENCODER_A, m2_encoder_interrupt, CHANGE);
      attachInterrupt(M2_ENCODER_B, m2_encoder_interrupt, CHANGE);
      break;
    case MOTOR3:
      motorType=STEPPER_MOTOR;
      attachInterrupt(M3_ENCODER_A, m3_encoder_interrupt, CHANGE);
      attachInterrupt(M3_ENCODER_B, m3_encoder_interrupt, CHANGE);
      break;
    case MOTOR4:
      motorType=STEPPER_MOTOR;
      attachInterrupt(M4_ENCODER_A, m4_encoder_interrupt, CHANGE);
      attachInterrupt(M4_ENCODER_B, m4_encoder_interrupt, CHANGE);
      break;
    case MOTOR5:
      motorType=SERVO_MOTOR;
      break;
    case MOTOR6:
      motorType=SERVO_MOTOR;
      break;
  }
}

/*
 * there could be background interrupt routines for all the motor types that wait for instructions...
 * so far we want the automatic control to be done through interrupts, and the thing that tells the
 * ISR how to move is the pid controller. so for manual control, i need another set of functions that
 * tell the ISR how to move. but for now I can just make simple code that does the thing, I suppose.
*/

void ArmMotor::budge(int rotationDir){
  // arranged in order of which motor is predicted to be controlled the most
  switch(motorType){
    case STEPPER_MOTOR:
      
      break;
    case DC_MOTOR:
      break;
    case SERVO_MOTOR:
      if(rotationDir==CLOCKWISE){
        //pwm signal one way
        
      }
      if(rotationDir==COUNTER_CLOCKWISE){
        //pwm signal other way

      }
      break;
  }
}

void ArmMotor::setDesiredAngle(float angle){
  // arranged in order of which motor is predicted to be controlled the most
  switch(motorType){
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
