#include "APA102.h"

#define START_FRAME_SIZE 4
SPISettings settingsA(100000000, MSBFIRST, SPI_MODE0); 

APA102::APA102(SPIClass* spi, int numOfLEDs): spi(spi), numOfLEDs(numOfLEDs){
  int sizeEndFrame = numOfLEDs/16;
  if (sizeEndFrame * 8 < (numOfLEDs / 2)) {
    sizeEndFrame++;
  }
  bufSize = START_FRAME_SIZE + numOfLEDs * 4 + sizeEndFrame;
  buf = (byte*)malloc(bufSize);
  for (int i = 0; i < START_FRAME_SIZE; i++) {
    buf[i] = 0;
  }
  for (int i = START_FRAME_SIZE; i < 4 * numOfLEDs + START_FRAME_SIZE; i+= 4) {
    buf[i + 0] = 0xE0;
    buf[i + 1] = 0;
    buf[i + 2] = 0;
    buf[i + 3] = 0;
  }
  for (int i = START_FRAME_SIZE + numOfLEDs * 4; i < bufSize; i++) {
    buf[i] = 255;
  }  
}
void APA102::send() {
  spi->beginTransaction(settingsA);
  for (int i = 0; i < bufSize; i++) {
    spi->transfer(buf[i]);
    Serial.print(buf[i]);
    Serial.print(" ");
  }
  Serial.println();

  spi->endTransaction();
}

void APA102::setColor(int index, int r, int g, int b, int brightness) {
  buf[4 * index + 4] = 0xE0 + constrain(brightness, 0, 31);
  buf[4 * index + 5] = constrain(b, 0, 255);
  buf[4 * index + 6] = constrain(g, 0, 255);
  buf[4 * index + 7] = constrain(r, 0, 255);
}

void APA102::setAll(int r, int g, int b, int brightness) {
  for (int i = 0; i < numOfLEDs; i++) {
    setColor(i, r, g, b, brightness);
  }
}

void APA102::setToOff() {
  setAll(0, 0, 0, 0);
}