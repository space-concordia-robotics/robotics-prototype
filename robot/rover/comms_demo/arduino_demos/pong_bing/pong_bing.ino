#include <Arduino.h>
/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);
}

// the loop routine runs over and over again forever:
void loop() {
//  // read the input on analog pin 0:

  int pingValue;
  Serial.write(pingValue);
  Serial.write(0);
  Serial.write(0x0A);

  delay(1000);
  

  int debugMessageID = 0;
  char* message = "bing";
  Serial.write(sizeof(message) + 1);
  Serial.write(message);
  Serial.write(0x0A);


  delay(1000);
  
  int motorsValue = 17;
  float motors[6];
  motors[0] = 0.0f;
  motors[1] = 1.25f;
  motors[2] = 1.375f;
  motors[3] = 1.425f;
  motors[4] = -2.25f;
  motors[5] = 2.0f;

  Serial.write(motorsValue);
  Serial.write(sizeof(motors));
  byte* motorsByte = (byte*)motors;
  Serial.write(motorsByte, sizeof(motors));
  Serial.write(0x0A);
  delay(1000);

}
