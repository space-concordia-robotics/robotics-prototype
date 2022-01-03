#include <Arduino.h>
uint32_t currentTime;

// the setup routine runs once when you press reset:
void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    pinMode(LED_BUILTIN,OUTPUT);
    currentTime = millis();
}
void blink(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    delay(500);

}
// the loop routine runs over and over again forever:
void loop() {
// read the input on analog pin 0:
if(millis() - currentTime > 2000){
    blink();
    currentTime = millis();
}

}
