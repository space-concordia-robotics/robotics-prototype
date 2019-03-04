#include <Arduino.h>

#define M2_PWM_PIN         30
#define M2_DIR_PIN         31 // for new driver
#define M2_ENCODER_PORT    GPIOA_PDIR
#define M2_ENCODER_SHIFT   CORE_PIN26_BIT
#define M2_ENCODER_A       26
#define M2_ENCODER_B       27

void ISRA(void) {
  Serial.println("beep");
}

void ISRB(void) {
  Serial.println("boop");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT); // pin 13
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200); Serial.setTimeout(10);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT); // for new driver
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
  pinMode(M2_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(M2_ENCODER_A, ISRA, CHANGE);
  attachInterrupt(M2_ENCODER_B, ISRB, CHANGE);

  digitalWrite(M2_DIR_PIN,LOW);
  analogWrite(M2_PWM_PIN,127);
}

void loop() {
  // put your main code here, to run repeatedly:

}
