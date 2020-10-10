#ifndef HELPERS_H
#define HELPERS_H
#include "Arduino.h"
#include <SoftwareSerial.h>
extern SoftwareSerial bluetooth;
extern bool devMode;
extern bool bluetoothMode;
class Helpers{

public:
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) ;

void ser_flush(void);

void toggleLed() ;

String getValue(String data, char separator, int index) ;


  static Helpers& get();

    
  void print(String msg);
  void println(String msg);
  void printres(float msg, int a);
  
};


#endif
