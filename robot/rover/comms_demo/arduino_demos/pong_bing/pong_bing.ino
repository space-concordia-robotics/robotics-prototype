#include <Arduino.h>
/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
#define M6_RL_PWM 2
#define M5_ML_PWM 3 //works
#define M4_FL_PWM 4 //workls

#define M3_RR_PWM 5
#define M2_MR_PWM 6
#define M1_FR_PWM 7


#define M6_RL_DIR 26
#define M5_ML_DIR 25
#define M4_FL_DIR 24

#define M3_RR_DIR 12
#define M2_MR_DIR 11
#define M1_FR_DIR 10 //Check this one


#define M6_RL_A 27
#define M6_RL_B 28

#define M5_ML_A 33
#define M5_ML_B 34

#define M4_FL_A 31
#define M4_FL_B 32

#define M3_RR_A 29
#define M3_RR_B 30

#define M2_MR_A 37
#define M2_MR_B 38

#define M1_FR_A 35
#define M1_FR_B 36


// the setup routine runs once when you press reset:
void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(57600);
    pinMode(LED_BUILTIN,OUTPUT);

    pinMode(M1_FR_PWM, OUTPUT); //works
    pinMode(M1_FR_DIR, OUTPUT); //nope

    pinMode(M2_MR_PWM, OUTPUT); //works
    pinMode(M2_MR_DIR, OUTPUT); //works

    pinMode(M3_RR_DIR, OUTPUT); //works
    pinMode(M3_RR_PWM, OUTPUT); //works

    pinMode(M4_FL_DIR, OUTPUT); //works
    pinMode(M4_FL_PWM, OUTPUT); //works

    pinMode(M5_ML_DIR, OUTPUT); //works
    pinMode(M5_ML_PWM, OUTPUT); //works

    pinMode(M6_RL_DIR, OUTPUT); //works
    pinMode(M6_RL_PWM, OUTPUT); //works



}

// the loop routine runs over and over again forever:
void loop() {
// read the input on analog pin 0:

//    digitalWrite(RF_DIR,HIGH);
//    analogWrite(RF_PWM,240); // Spins right middle

    digitalWrite(M5_ML_DIR,LOW);
    analogWrite(M5_ML_PWM,80);

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
  motors[1] = 1.25f;/home/roversim/Downloads/WheelsCommandCenter.h
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
