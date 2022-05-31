#include "../include/PinSetup.h"

#include <Arduino.h>

void pinSetup(void) {
  pinMode(LED_BUILTIN, OUTPUT);  // pin 13, heartbeat
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(V_SENSE, INPUT);  // battery sensing

  // DC motors

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);

  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);

  pinMode(M3_PWM_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);

  pinMode(M4_DIR_PIN, OUTPUT);
  pinMode(M4_PWM_PIN, OUTPUT);
}
