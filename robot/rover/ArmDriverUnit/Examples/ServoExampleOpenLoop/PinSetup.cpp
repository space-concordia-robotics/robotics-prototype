#include <Arduino.h>
#include "PinSetup.h"

void pinSetup(void) {
  pinMode(LED_BUILTIN, OUTPUT); // pin 13
  digitalWrite(LED_BUILTIN, HIGH);
  
  // servo
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, SERVO_STOP);
}
