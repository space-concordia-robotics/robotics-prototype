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

    Serial.print(0x02);
    delay(500);


}
