#define M5_PWM_PIN         35
/*
  // 33&34 are on port E with bits 24&25 respectively
  #define M5_ENCODER_PORT    GPIOE_PDIR
  #define M5_ENCODER_A       33
  #define M5_ENCODER_B       34
*/
#define M5_GEAR_RATIO      84.0/12.0 // there is a ratio here that I don't know yet
//#define M5_MINIMUM_ANGLE
//#define M5_MAXIMUM_ANGLE
// no angle limits because this one can be used as a screwdriver

#define M6_PWM_PIN         36
/*
  // 37&38 are on port C with bits 10&11 respectively
  #define M6_ENCODER_PORT    GPIOC_PDIR
  #define M6_ENCODER_A       37
  #define M6_ENCODER_B       38
*/
#define M6_GEAR_RATIO      40.0/12.0 // there is a ratio here that I don't know yet
#define M6_MINIMUM_ANGLE   -75.0 //-120.0
#define M6_MAXIMUM_ANGLE   75.0 // 150.0 //30.0

#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  // put your setup code here, to run once:
  pinMode(M5_PWM_PIN, OUTPUT);
  /*
    pinMode(M5_ENCODER_A, LIM_SWITCH_PULLSTATE);
    pinMode(M5_ENCODER_B, LIM_SWITCH_PULLSTATE);
  */

  pinMode(M6_PWM_PIN, OUTPUT);
  /*
    pinMode(M6_ENCODER_A, LIM_SWITCH_PULLSTATE);
    pinMode(M6_ENCODER_B, LIM_SWITCH_PULLSTATE);
  */

  servo1.attach(M5_PWM_PIN);
  servo2.attach(M6_PWM_PIN);

  servo1.writeMicroseconds(2000);
  servo2.writeMicroseconds(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
