#include <Arduino.h>
/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
// direction pins
#define RF_DIR   2
#define RM_DIR   11
#define RB_DIR   26//12
#define LF_DIR   24
#define LM_DIR   25
#define LB_DIR   12//26

// pwm pins
#define RF_PWM   3
#define RM_PWM   4
#define RB_PWM   8//5
#define LF_PWM   6
#define LM_PWM   7
#define LB_PWM   5//8


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
    pinMode(LED_BUILTIN,OUTPUT);

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

// the loop routine runs over and over again forever:
void loop() {
// read the input on analog pin 0:

//    digitalWrite(RF_DIR,HIGH);
//    analogWrite(RF_PWM,240); // Spins right middle

    digitalWrite(RM_DIR,HIGH);
    analogWrite(RM_PWM,130); // spins right front

//    digitalWrite(RB_DIR,HIGH);
//    analogWrite(RB_PWM,160); // does nothing

    //digitalWrite(LF_DIR,HIGH);
    //analogWrite(LF_PWM,160); // spins left middle

    //digitalWrite(LM_DIR,HIGH);
   // analogWrite(LM_PWM,230); // nothing

  //  digitalWrite(LB_DIR,HIGH);
   // analogWrite(LB_PWM,230); // nothing



  int pingValue = 1;
  /*Serial.write(pingValue);
  Serial.write(0);
  Serial.write(0);
  Serial.write(0x0A); */
  digitalWrite(LED_BUILTIN,HIGH);

  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
    /*
  int debugMessageID = 0;
  const char* message = "bing bang boom!!";
  Serial.write(debugMessageID);
  Serial.write(0);
  Serial.write(strlen(message));
  Serial.write(message);
  Serial.write(0x0A);


  delay(1000);
  
  int motorsValue = 2;
  float motors[6];
  motors[0] = 0.0f;
  motors[1] = 1.25f;
  motors[2] = 1.375f;
  motors[3] = 1.425f;
  motors[4] = -2.25f;
  motors[5] = 2.0f;

  Serial.write(motorsValue);
  Serial.write(0);
  Serial.write(sizeof(motors));
  byte* motorsByte = (byte*)motors;
  Serial.write(motorsByte, sizeof(motors));
  Serial.write(0x0A);
  delay(1000);
    */
}
