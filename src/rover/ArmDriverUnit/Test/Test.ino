#include "Benis.h"

#define BAUD_RATE 115200      //serial baud rate
#define BUFFER_SIZE 100     //size of the buffer for the serial commands

void setup() {
  Serial.begin(BAUD_RATE);
  Benis benis(5);
  benis.bepis=3;
  /*
  Motor motor1 = new Motor(MOTOR1);
  Motor motor2 = new Motor(MOTOR2);
  Motor motor3 = new Motor(MOTOR3);
  Motor motor4 = new Motor(MOTOR4);
  Motor motor5 = new Motor(MOTOR5);
  Motor motor6 = new Motor(MOTOR6);
  */
}

void loop() {
  
}




















