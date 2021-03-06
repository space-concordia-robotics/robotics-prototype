#include <Arduino.h>
#include <Servo.h>

#define COUNTER_CLOCKWISE  1
#define CLOCKWISE         -1
#define M2_LIM_PLUS       26
#define M2_LIM_PLUS_SHIFT  CORE_PIN26_BIT
#define M2_LIM_PLUS_PORT  GPIOA_PDIR
#define M2_LIM_MINUS      27
#define M2_LIM_MINUS_SHIFT  CORE_PIN27_BIT
#define M2_LIM_MINUS_PORT GPIOA_PDIR
#define M5_SERVO_PWM      35
#define SERVO_STOP        1500
#define TURN_CCW          1200
#define TURN_CW           1800
#define TRIGGER_DELAY     10

volatile bool triggered = false;
bool actualPress = false;
volatile int triggerState = 0;
int limitSwitchState = 0;
elapsedMillis sinceTrigger;
Servo servo;

void limSwitchPlus(void) {
  // the trigger occurs upon the first contact before the bouncing and starts counting until the time limit
  // at which point the trigger is reset in the main loop

  // but it only gets triggered on a rising edge, so triggerState should be 0 before the trigger
  // if triggerState was not 0 it means that it's a falling edge
  if (!triggered && triggerState == 0) {
    triggered = true;
    sinceTrigger = 0;
  }
  int pinState = (M2_LIM_PLUS_PORT >> M2_LIM_PLUS_SHIFT ) & 1;
  if (pinState == 0) {
    triggerState = COUNTER_CLOCKWISE;
    //Serial.println("plus");
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
  int pinState = (M2_LIM_MINUS_PORT >> M2_LIM_MINUS_SHIFT ) & 1;
  if (pinState == 0) {
    triggerState = CLOCKWISE;
    //Serial.println("minus");
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
  pinMode(M2_LIM_PLUS, INPUT_PULLUP);
  pinMode(M2_LIM_MINUS, INPUT_PULLUP);
  attachInterrupt(M2_LIM_PLUS, limSwitchPlus, CHANGE);
  attachInterrupt(M2_LIM_MINUS, limSwitchMinus, CHANGE);
  servo.attach(M5_SERVO_PWM);
  servo.writeMicroseconds(SERVO_STOP);
}

void loop() {
  // this works but doesn't consider the possibility of both switches
  // being triggered at the same time which shouldn't ever actually happen
  if (triggered) {
    if (sinceTrigger >= TRIGGER_DELAY) {
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
    //Serial.println(limitSwitchState);
    if (limitSwitchState == COUNTER_CLOCKWISE) {
      //Serial.println("ccw");
      servo.writeMicroseconds(TURN_CCW);
      while (timer < 1000) {
        ;
      }
    }
    if (limitSwitchState == CLOCKWISE) {
      //Serial.println("cw");
      servo.writeMicroseconds(TURN_CW);
      while (timer < 1000) {
        ;
      }
    }
    // now that the behaviour is complete we can reset these in wait for the next trigger to be confirmed
    actualPress = false;
    limitSwitchState = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    servo.writeMicroseconds(SERVO_STOP);
  }
}
