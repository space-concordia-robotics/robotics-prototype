#include <Arduino.h>
uint32_t currentTime;
#define M6_RL_PWM 2
#define M5_ML_PWM 3
#define M4_FL_PWM 4

#define M3_RR_PWM 5
#define M2_MR_PWM 6
#define M1_FR_PWM 7


#define M6_RL_DIR 26
#define M5_ML_DIR 25
#define M4_FL_DIR 24

#define M3_RR_DIR 12
#define M2_MR_DIR 11
#define M1_FR_DIR 8


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

void initMotorPins(){
    pinMode(M1_FR_DIR,OUTPUT);
    pinMode(M1_FR_PWM,OUTPUT);
    pinMode(M2_MR_DIR,OUTPUT);
    pinMode(M2_MR_PWM,OUTPUT);
    pinMode(M3_RR_DIR,OUTPUT);
    pinMode(M3_RR_PWM,OUTPUT);
    pinMode(M4_FL_DIR,OUTPUT);
    pinMode(M4_FL_PWM,OUTPUT);
    pinMode(M5_ML_DIR,OUTPUT);
    pinMode(M5_ML_DIR,OUTPUT);
    pinMode(M6_RL_PWM,OUTPUT);
    pinMode(M6_RL_PWM,OUTPUT);
}

void initEncoderPins(){
    pinMode(M1_FR_A,INPUT_PULLUP);
    pinMode(M1_FR_B,INPUT_PULLUP);
    pinMode(M2_MR_A,INPUT_PULLUP);
    pinMode(M2_MR_B,INPUT_PULLUP);
    pinMode(M3_RR_A,INPUT_PULLUP);
    pinMode(M3_RR_B,INPUT_PULLUP);
    pinMode(M4_FL_A,INPUT_PULLUP);
    pinMode(M4_FL_B,INPUT_PULLUP);
    pinMode(M5_ML_A,INPUT_PULLUP);
    pinMode(M5_ML_B,INPUT_PULLUP);
    pinMode(M6_RL_A,INPUT_PULLUP);
    pinMode(M6_RL_B,INPUT_PULLUP);
}

void IRC_M1(){
    Serial.write(0x01);
}

void IRC_M2(){
    Serial.write(0x02);
}
void IRC_M3(){
    Serial.write(0x03);
}
void IRC_M4(){
    Serial.write(0x04);
}
void IRC_M5(){
    Serial.write(0x05);
}
void IRC_M6(){
    Serial.write(0x06);
}
attachInterrupts(){
    attachInterrupt(M1_FR_A,IRC_M1,CHANGE);
    attachInterrupt(M1_FR_B,IRC_M1,CHANGE);

    attachInterrupt(M2_MR_A,IRC_M2,CHANGE);
    attachInterrupt(M2_MR_B,IRC_M2,CHANGE);

    attachInterrupt(M3_RR_A,IRC_M3,CHANGE);
    attachInterrupt(M3_RR_B,IRC_M3,CHANGE);

    attachInterrupt(M4_FL_A,IRC_M4,CHANGE);
    attachInterrupt(M4_FL_B,IRC_M4,CHANGE);

    attachInterrupt(M5_ML_A,IRC_M5,CHANGE);
    attachInterrupt(M5_ML_B,IRC_M5,CHANGE);

    attachInterrupt(M6_RL_A,IRC_M6,CHANGE);
    attachInterrupt(M6_RL_B,IRC_M6,CHANGE);
}
// the setup routine runs once when you press reset:
void setup() {

    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    pinMode(LED_BUILTIN,OUTPUT);

    initMotorPins();
    initEncoderPins();

    attachInterrupts();


    currentTime = millis();
}
void blink(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    delay(500);

}
// the loop routine runs over and over again forever:
void loop() {
// read the input on analog pin 0:
if(millis() - currentTime > 2000){
    blink();

}

}
