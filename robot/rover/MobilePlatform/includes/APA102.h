#ifndef APA102_H
#define APA102_h

#include "Arduino.h"
#include "SPI.h"

class APA102 {
public:
  APA102(SPIClass* spi, int numOfLEDs);
  void send();
  void setColor(int index, int r, int g, int b, int brightness = 31);
  void setAll(int r, int g, int b, int brightness = 31);
  void setToOff();
 private:
  SPIClass *spi;
  int numOfLEDs;
  int bufSize;
  byte* buf;
};

#endif