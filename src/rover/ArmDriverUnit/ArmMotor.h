#ifndef ARMMOTOR_H
#define ARMMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
//#include "AbtinEncoder.h"

/*
#define DC_MOTOR 0
#define STEPPER_MOTOR 1
#define SERVO_MOTOR 2

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
*/

/*
#define M1_STEP_HIGH        PORTD |=  0b10000000;
#define M1_STEP_LOW         PORTD &= ~0b10000000;

#define M2_STEP_HIGH        PORTD |=  0b00100000;
#define M2_STEP_LOW         PORTD &= ~0b00100000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);
*/

enum motor_code {MOTOR1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6};
enum motor_type {DC_MOTOR,STEPPER_MOTOR,SERVO_MOTOR};
enum motor_direction {CLOCKWISE,COUNTER_CLOCKWISE};
const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; //quadrature encoder matrix

void m1_encoder_interrupt();
void m2_encoder_interrupt();
void m3_encoder_interrupt();
void m4_encoder_interrupt();
void m5_encoder_interrupt();
void m6_encoder_interrupt();

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
		volatile int encoderCount;
    
    ArmMotor(int code);
    //void init();
		void update();
		
		void setMotorSpeed();
		void setDesiredAngle();
		float getCurrentAngle();
	private:
		int motorCode;
		int motorType;
		float currentAngle;
		float desiredAngle;
		bool isMovementDone;
		
		void changeMotorAngle(char direction, char speed);
} motor1(MOTOR1),motor2(MOTOR2),motor3(MOTOR3),motor4(MOTOR4),motor5(MOTOR5),motor6(MOTOR6);;

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

void ArmMotor:moveMotor(void){
	// arranged in order of which motor is predicted to be controlled the most
	switch(motorType){
		case STEPPER_MOTOR:
			
			break;
		case DC_MOTOR:
			break;
		case SERVO_MOTOR:
			if(direction==CLOCKWISE){
				//pwm signal one way
				analogWrite(
			}
			if(direction==COUNTER_CLOCKWISE){
				//pwm signal other way
			}
			break;
	}
}

void ArmMotor::changeMotorAngle(char direction, char speed){
	// arranged in order of which motor is predicted to be controlled the most
	switch(motorType){
		case STEPPER_MOTOR:
			break;
		case DC_MOTOR:
			break;
		case SERVO_MOTOR:
			if(direction==CLOCKWISE){
				//pwm signal one way
			}
			if(direction==COUNTER_CLOCKWISE){
				//pwm signal other way
			}
			break;
	}
}

void m1_encoder_interrupt(){
  static unsigned int oldM1EncoderState = 0;
  oldM1EncoderState <<= 2;  //move by two bits (multiply by 4);
  //read all bits on D register. shift ro right
  //so pin 2 and 3 are now the lowest bits
  //then AND this with 0X03 (0000 0011) to zero everything else
  //then OR this with the last encoder state to get a 4 byte number
  oldM1EncoderState |= ((PIND >> 2) & 0x03);
  //AND this number with 0X0F to make sure its a
  //4 bit unsigned number (0 to 15 decimal)
  //then use that number as an index from the array to add or deduct a 1 from the
  //count
  motor1.encoderCount += dir[(oldM1EncoderState & 0x0F)];
}

void m2_encoder_interrupt(){
  static unsigned int oldM2EncoderState = 0;
  oldM2EncoderState <<= 2;  //move by two bits (multiply by 4);
  oldM2EncoderState |= ((PIND >> 2) & 0x03);
  motor2.encoderCount += dir[(oldM2EncoderState & 0x0F)];
}

void m3_encoder_interrupt(){
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;  //move by two bits (multiply by 4);
  oldEncoderState |= ((PIND >> 2) & 0x03);
  motor3.encoderCount += dir[(oldEncoderState & 0x0F)];
}

void m4_encoder_interrupt(){
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;  //move by two bits (multiply by 4);
  oldEncoderState |= ((PIND >> 2) & 0x03);
  motor4.encoderCount += dir[(oldEncoderState & 0x0F)];
}

#endif
