#ifndef APA102_H
#define APA102_h

#include "Arduino.h"

class APA102 {
public:
  APA102(int numOfLEDs, int mosiPin, int clockPin);
  void send();
  void setColor(int index, int r, int g, int b, int brightness = 31);
  void setAll(int r, int g, int b, int brightness = 31);
  void setToOff();
 private:
  int numOfLEDs;
  int bufSize;
  int mosiPin, clockPin;
  byte* buf;
};

#endif