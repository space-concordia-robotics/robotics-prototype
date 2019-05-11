#include <Servo.h>

int servoPin = 3;
// defaults are 544 and 2400
int minDuration = 544;
int maxDuration = 2400;

Servo servo1;

void setup() {
  // put your setup code here, to run once:
  //pinMode(servoPin, OUTPUT);
  servo1.attach(servoPin, minDuration, maxDuration);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial.available()) {
    int data = (Serial.readString()).toInt();
    Serial.println(data);
    //if (data >= 0 && data <= 255) {
    if (data >= minDuration && data <= maxDuration){
      //servo1.write(data);
      servo1.writeMicroseconds(data);
      //analogWrite(servoPin, data);
    }
  }
}
