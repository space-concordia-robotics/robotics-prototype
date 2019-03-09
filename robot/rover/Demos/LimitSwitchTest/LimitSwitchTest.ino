#include <Arduino.h>

#define COUNTER_CLOCKWISE  1
#define CLOCKWISE         -1
#define M1_LIM_PLUS       31
#define M1_LIM_MINUS      32

volatile bool triggered = false;
bool actualPress = false;
volatile int triggerState = 0;
int limitSwitchState = 0;
elapsedMillis sinceTrigger;

void limSwitchPlus(void) {
  // the trigger occurs upon the first contact before the bouncing and starts counting until the time limit
  // at which point the trigger is reset in the main loop
  
  // but it only gets triggered on a rising edge, so triggerState should be 0 before the trigger
  // if triggerState was not 0 it means that it's a falling edge
  if (!triggered && triggerState == 0) {
    triggered = true;
    sinceTrigger = 0;
  }
  // regardless of whether it's triggered, the triggerState will change from
  // 0 to ccw or ccw to 0 in this ISR
  if (triggerState == 0) {
    triggerState = COUNTER_CLOCKWISE;
  }
  else {
    triggerState = 0;
  }
}

void limSwitchMinus(void) {
  if (!triggered && triggerState == 0) {
    triggered = true;
    sinceTrigger = 0;
  }
  if (triggerState == 0) {
    triggerState = CLOCKWISE;
  }
  else {
    triggerState = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT); // pin 13
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200); Serial.setTimeout(10);
  pinMode(M1_LIM_PLUS, INPUT_PULLUP);
  pinMode(M1_LIM_MINUS, INPUT_PULLUP);
  attachInterrupt(M1_LIM_PLUS, limSwitchPlus, CHANGE);
  attachInterrupt(M1_LIM_MINUS, limSwitchMinus, CHANGE);
}

void loop() {
  // this works but doesn't consider the possibility of both switches
  // being triggered at the same time which shouldn't ever actually happen
  if (triggered) {
    if (sinceTrigger >= 15) {
      // if the last interrupt was a press (meaning it's stabilized and in contact)
      // then there's a real press
      if (triggerState != 0) {
        actualPress = true;
        limitSwitchState = triggerState;
      }
      // otherwise it's not a real press
      // so the limit switch state should stay whatever it used to be
      // and so should actualPress
      else {
        ;
      }
      // either way, we should reset the triggered bool in wait for the next trigger
      triggered = false;
    }
  }
  if (actualPress) {
    digitalWrite(LED_BUILTIN, LOW);
    elapsedMillis timer;
    if (limitSwitchState = COUNTER_CLOCKWISE) {
      // blocking loop is fine for testing
      while (timer < 1000) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    if (limitSwitchState == CLOCKWISE) {
      // blocking loop is fine for testing
      while (timer < 2000) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(400);
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    // now that the behaviour is complete we can reset these in wait for the next trigger to be confirmed
    actualPress = false;
    limitSwitchState = 0;
  }
}
