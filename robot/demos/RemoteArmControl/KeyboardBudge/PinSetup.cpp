#include "PinSetup.h"

#include <Arduino.h>

void pinSetup(void) {
  pinMode(LED_BUILTIN, OUTPUT);  // pin 13
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_STEP_PIN, OUTPUT);

  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);

  pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M3_STEP_PIN, OUTPUT);

  pinMode(M4_DIR_PIN, OUTPUT);
  pinMode(M4_STEP_PIN, OUTPUT);
}