#include <FastLED.h>
#include <Servo.h>
#include "commands.h"

#define LED_INTERVAL 1000

#define INDICATOR_LED_PIN 16
#define EUREKA_LED_INTERVAL 2000
#define INDICATOR_LED_COUNT 16
#define INDICATOR_LED_BRIGHTNESS 30
CRGB indicatorLedArray[INDICATOR_LED_COUNT];

// Test servo w/ LED
Servo myservo;
int servoPos = 0;
bool hack = true; // Servo is set to "sentry mode" when it reaches 180 deg this will be set to false, at 0 deg will be set true
/* This "hack" is just to test the servo without using for-loops to see if the use of the LEDs is blocking the servo from working. */

Commands Cmds;  
String input;

elapsedMillis sinceLedToggle; // time since led off
elapsedMillis sinceEurekaToggle; // time since led on
//elapsedMicros delayTime; // time delay for servo/

void setup() {
    Serial.begin(9600);
    FastLED.addLeds<NEOPIXEL, INDICATOR_LED_PIN>(indicatorLedArray, INDICATOR_LED_COUNT);
    FastLED.setBrightness(INDICATOR_LED_BRIGHTNESS);

    myservo.attach(9);

}

void loop() {
    if(Serial.available()){
        input = Serial.readStringUntil("\n");
        Serial.println(input);
        Cmds.handler(input); 
        if(Cmds.isLedOn){
          setIndicatorMode(Cmds.indicatorRgbArray[0], Cmds.indicatorRgbArray[1], Cmds.indicatorRgbArray[2]);
        }
        else{
          FastLED.clear(); // Sets RGB values to (0,0,0)
          FastLED.show(); // Push RGB values
        }
    }
    // fix eureka blink
    if(Cmds.indicatorRgbArray[1] == 255 && Cmds.isLedOn){
      if(sinceLedToggle > LED_INTERVAL){
        setIndicatorMode(0,255,0);
      }
      if(sinceLedToggle > EUREKA_LED_INTERVAL){
        FastLED.clear(); // Sets RGB values to (0,0,0)
        FastLED.show(); // Push RGB values
        sinceLedToggle = 0;
      }
    }

// hack servo sentry code
 /*   
    if(hack){
      if(delayTime > 25){
          myservo.write(servoPos);
          delayTime = 0;
          servoPos++;
      }
      if(servoPos == 180){
        hack = false;
      }
    }
    else{
      if(delayTime > 25){
          myservo.write(servoPos);
          delayTime = 0;
          servoPos--;
      }      
      if(servoPos == 0){
        hack = true;
      }
    }
*/
// for-loop servo sentry code
/*
    for( servoPos = 0; servoPos <= 180; servoPos += 1){
      if(delayTime > 15){
          myservo.write(servoPos);
          delayTime = 0;
      }
    }
    for(servoPos = 180; servoPos >= 0; servoPos -= 1){
      if(delayTime > 15){
          myservo.write(servoPos);
          delayTime = 0;
      }
    }
}
*/
void setIndicatorMode(int r, int g, int b){
      for(uint16_t i = 0; i < INDICATOR_LED_COUNT; i++){
        indicatorLedArray[i].setRGB(r,g,b);
      }
      FastLED.show(); // set led on
}
