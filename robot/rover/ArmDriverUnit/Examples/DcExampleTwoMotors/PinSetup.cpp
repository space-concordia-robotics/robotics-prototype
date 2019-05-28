#include <Arduino.h>
#include "PinSetup.h"

void pinSetup(void) {
  pinMode(LED_BUILTIN, OUTPUT); // pin 13
  digitalWrite(LED_BUILTIN, HIGH);
  
  // DC motor

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT); // for new driver
  //analogWrite(M1_PWM_PIN, DC_STOP); // for sabertooth
  //pinMode(M1_UART_TX, OUTPUT); // for sabertooth
  //pinMode(M1_UART_RX, INPUT); // for sabertooth
  pinMode(M1_ENCODER_A, INPUT_PULLUP);
  pinMode(M1_ENCODER_B, INPUT_PULLUP);
  pinMode(M1_LIMIT_SW_CW, LIM_SWITCH_PULLSTATE);
  pinMode(M1_LIMIT_SW_CCW, LIM_SWITCH_PULLSTATE);

pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT); // for new driver
  //analogWrite(M2_PWM_PIN, DC_STOP); // for sabertooth
  //pinMode(M2_UART_TX, OUTPUT); // for sabertooth
  //pinMode(M2_UART_RX, INPUT); // for sabertooth
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
  pinMode(M2_ENCODER_B, INPUT_PULLUP);
  pinMode(M2_LIMIT_SW_FLEX, LIM_SWITCH_PULLSTATE);
  pinMode(M2_LIMIT_SW_EXTEND, LIM_SWITCH_PULLSTATE);
}
