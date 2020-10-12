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
//assumes Serial.begin() already in main code
//Will print a battery voltage to the screen
void Helpers::vbatt_read(int v_sense_pin) {
  float vsense = analogRead(v_sense_pin);
  vsense *= 0.003225806; //convert to 3.3V reference from analog values (3.3/1023=0.003225806)
  //voltage divider backwards (vsense *(r1+r2)/r2 = vsense * (10k+2k)/2k = vsense*6)
  float vbatt = vsense * 6.0;

  Helpers::get().print("ASTRO Battery voltage: ");
  Helpers::get().println(vbatt);

  if (vbatt < 12.0) {
    Helpers::get().println("ASTRO WARNING! BATTERY VOLTAGE IS LOW! DISCONNECT IMMEDIATELY");
  }
  else if (vbatt > 16.8) {
    Helpers::get().println("ASTRO WARNING! BATTERY VOLTAGE IS HIGH! DISCONNECT IMMEDIATELY!");
  }
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
