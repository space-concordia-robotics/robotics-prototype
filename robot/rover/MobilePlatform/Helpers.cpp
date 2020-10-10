#include "Helpers.h"
bool devMode;
bool bluetoothMode;

Helpers& Helpers::get(){
  static Helpers instance;
  return instance;
}

float Helpers::mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Helpers::ser_flush(){
   if (devMode) {
        while (Serial.available()) {
            Serial.read();
        }
    }
    else {
        while (Serial1.available()) {
            Serial1.read();
        }
    }
}
void Helpers::toggleLed(){
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
String Helpers::getValue(String data, char separator, int index){
    int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void Helpers::print(String msg) {
  if (devMode) {
    Serial.print(msg);
    if (bluetoothMode) {
      bluetooth.print(msg);
      //            delay(50);
    }
  }
  else {
    Serial1.print(msg);
    if (bluetoothMode) {
      bluetooth.print(msg);
      //            delay(50);
    }
  }
}
void Helpers::println(String msg) {
  if (devMode) {
    Serial.println(msg);
    if (bluetoothMode) {
      bluetooth.print(msg);
      //            delay(50);
    }
  }
  else {
    Serial1.println(msg);
    if (bluetoothMode) {
      bluetooth.print(msg);
      //            delay(50);
    }
  }
}
void Helpers::printres(float msg, int a) {
  if (devMode) {
    Serial.print(msg, a);
    if (bluetoothMode) {
      bluetooth.print(msg);
      //            delay(50);
    }
  }
  else {
    Serial1.print(msg, a);
    if (bluetoothMode) {
      bluetooth.print(msg);
      //            delay(50);
    }
  }
}
