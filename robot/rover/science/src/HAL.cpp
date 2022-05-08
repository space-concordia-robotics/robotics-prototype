#include "include/HAL.h"

#include "Arduino.h"
#include "include/SciencePinSetup.h"

void HAL::pinSetup() {
  pinMode(LIMIT_SW_0, INPUT);
  pinMode(LIMIT_SW_1, INPUT);
  pinMode(LIMIT_SW_2, INPUT);
  pinMode(LIMIT_SW_3, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LASER, OUTPUT);

  pinMode(HV_POWER_IRQ, INPUT);
  pinMode(LV_POWER_IRQ, INPUT);
  pinMode(HV_POWER_ACK, OUTPUT);
  pinMode(LV_POWER_ACK, OUTPUT);
  Serial5.begin(57600);  // debug serial
}

void HAL::estop() {
  laser(0);
  pump(0, 0);
  // TODO: check if this is really how to stop the servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo(i, 127);
  }
  theSmartServo.writeActionCommand(CAROUSEL_MOTOR_ID, "H");
}

void HAL::laser(uint8_t on) {
  if (on == 1) {
    digitalWrite(LASER, HIGH);
  } else {
    digitalWrite(LASER, LOW);
  }
}

void HAL::pump(uint8_t pwm, uint8_t forward) {
  if (forward) {
    analogWrite(PUMP_FORWARD, pwm);
    analogWrite(PUMP_BACK, 0);
  } else {
    analogWrite(PUMP_BACK, pwm);
    analogWrite(PUMP_FORWARD, 0);
  }
}

void HAL::servo(uint8_t servoId, uint8_t pwm) {
  if (servoId >= NUM_SERVOS) {
    Serial5.print("Cannot set servo with id ");
    Serial5.println(servoId);
  } else {
    int servoPin = SERVO_0 + servoId;
    analogWrite(servoPin, pwm);
  }
}

LSSServoMotor HAL::theSmartServo(&Serial2);

LSSServoMotor* HAL::smartServo() { return &theSmartServo; }

uint8_t HAL::readLimitSwitch(uint8_t switchId) {
  switch (switchId) {
    case 0:
      return digitalRead(LIMIT_SW_0);
    case 1:
      return digitalRead(LIMIT_SW_0);
    case 2:
      return digitalRead(LIMIT_SW_0);
    case 3:
      return digitalRead(LIMIT_SW_0);
    default:
      Serial5.print("Cannot read limit switch with ID");
      Serial5.println(switchId);
  }
}

void HAL::handleSwitches(int switchId, int switchPin) {
  int value = digitalRead(switchPin);
  for (size_t i = 0; i < powerCallbacks.size(); i++) {
    if (switchCallbacks[i].switchId == switchId) {
      switchCallbacks[i].cb(value);
    }
  }
}

void HAL::handleSwitch0() { handleSwitches(0, LIMIT_SW_0); }
void HAL::handleSwitch1() { handleSwitches(1, LIMIT_SW_1); }
void HAL::handleSwitch2() { handleSwitches(2, LIMIT_SW_2); }
void HAL::handleSwitch3() { handleSwitches(3, LIMIT_SW_3); }

void HAL::addLimitSwitchCallback(int switchId, pinCallback callback) {
  switch (switchId) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(LIMIT_SW_0), handleSwitch0, CHANGE);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(LIMIT_SW_1), handleSwitch1, CHANGE);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(LIMIT_SW_2), handleSwitch2, CHANGE);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(LIMIT_SW_3), handleSwitch3, CHANGE);
      break;
  }
  noInterrupts();
  switchCallbacks[switchCallbacks.size()] = SwitchCallback(switchId, callback);
  interrupts();
}

void HAL::handlePower() {
  for (size_t i = 0; i < powerCallbacks.size(); i++) {
    powerCallbacks[i]();
  }
}

void HAL::addPowerCallback(voidCallback callback) {
  attachInterrupt(digitalPinToInterrupt(HV_POWER_IRQ), handlePower, RISING);
  attachInterrupt(digitalPinToInterrupt(LV_POWER_IRQ), handlePower, RISING);
  powerCallbacks[powerCallbacks.size()] = callback;
}
