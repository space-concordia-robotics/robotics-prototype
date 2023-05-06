#include "include/HAL.h"

#include <Servo.h>

#include "Arduino.h"

// default instance of switch callback. someone tell me how to make this better
// pls
SwitchCallback s(-1, [](int b, void *user_ptr) {}, nullptr);
// Static members
// both these C arrays are initialized empty
SwitchCallback HAL::switchCallbacks[5] = {s, s, s, s, s};
voidCallback HAL::powerCallbacks[3] = {[]() {}, []() {}, []() {}};
int HAL::switchCallbacksSize = 0;
int HAL::powerCallbacksSize = 0;
etl::array<Servo*, NUM_SERVOS> HAL::servos = {};

void HAL::pinSetup() {
  pinMode(LIMIT_SW_0, INPUT_PULLUP);
  pinMode(LIMIT_SW_1, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  pinMode(HV_POWER_IRQ, INPUT);
  pinMode(LV_POWER_IRQ, INPUT);
  pinMode(HV_POWER_ACK, OUTPUT);
  pinMode(LV_POWER_ACK, OUTPUT);
  
  // For smart servo
  Serial5.begin(115200);
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    Servo* newServo = new Servo();
    servos[i] = newServo;
    servos[i]->attach(SERVO_0_PIN + i);
  }

  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_0), handleSwitch0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_1), handleSwitch1, CHANGE);

  attachInterrupt(digitalPinToInterrupt(HV_POWER_IRQ), handlePower, RISING);
  attachInterrupt(digitalPinToInterrupt(LV_POWER_IRQ), handlePower, RISING);
}

void HAL::estop() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo(i, 90);
  }
}

void HAL::servo(uint8_t servoId, uint8_t angle) {
  if (servoId >= NUM_SERVOS) {
    //Serial5.print("Cannot set servo with id ");
    //Serial5.println(servoId);
  } else {
    servos[servoId]->write(angle);
  }
}


int8_t HAL::readLimitSwitch(uint8_t switchId) {
  switch (switchId) {
    case 0:
      return !digitalRead(LIMIT_SW_0);
    case 1:
      return !digitalRead(LIMIT_SW_1);
    default:
      //Serial5.print("Cannot read limit switch with ID");
      //Serial5.println(switchId);
      return -1;
  }
}

void HAL::handleSwitches(int switchId) {
  int value = readLimitSwitch(switchId);

  for (size_t i = 0; i < switchCallbacksSize; i++) {
    if (switchCallbacks[i].switchId == switchId) {
      //digitalWrite(LED, !digitalRead(LED));
      switchCallbacks[i].cb(value, switchCallbacks[i].user_ptr);
    }
  }
}

void HAL::handleSwitch0() { handleSwitches(0); }
void HAL::handleSwitch1() { handleSwitches(1); }

void HAL::addLimitSwitchCallback(int switchId, pinCallback callback, void *user_ptr) {
  if (switchCallbacksSize < (sizeof(switchCallbacks) / sizeof(switchCallbacks[0]))) {
    SwitchCallback cb(switchId, callback, user_ptr);
    noInterrupts();
    switchCallbacks[switchCallbacksSize++] = cb;
    interrupts();
  } else {
    // TODO add feedback here once implemented.
  }
}

void HAL::handlePower() {
  for (size_t i = 0; i < powerCallbacksSize; i++) {
    powerCallbacks[i]();
  }
}

void HAL::addPowerCallback(voidCallback callback) {
  if (powerCallbacksSize < sizeof powerCallbacks) {
    powerCallbacks[powerCallbacksSize++] = callback;
  } else {
    // TODO add feedback once that's implemented.
  }
}
