#include <Arduino.h>

//#define M2_ENCODER_PORT    GPIOA_PDIR
//#define M2_ENCODER_SHIFT   CORE_PIN26_BIT
#define M2_ENCODER_A       26
#define M2_ENCODER_B       27
int pwmpin = 30;
int dirpin = 31;
float countablesPerRev = 9053.328; // every changing edge
//float gear = 188.611;
volatile unsigned long encoderCount = 0;
float revolutions = 0;
float gearAngle = 0;
int pwmval = 64;
char serialBuffer[100];
elapsedMillis timer;

void ISR(void) {
  encoderCount++;
}

void printCommands(void) {
  Serial.println("start: start dc motor");
  Serial.println("stop: stop dc motor");
  Serial.println("speed: set speet in pwm");
  Serial.println("fwd: set direction fwd");
  Serial.println("back: set direction back");
  Serial.println("help: print commands list");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT); // pin 13
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200); Serial.setTimeout(10);
  pinMode(pwmpin, OUTPUT);
  pinMode(dirpin, OUTPUT); // for new driver
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
  pinMode(M2_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(M2_ENCODER_A, ISR, CHANGE);
  attachInterrupt(M2_ENCODER_B, ISR, CHANGE);

  digitalWrite(dirpin, LOW);
  analogWrite(pwmpin, 0);
  printCommands();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) { // if a message was received
    Serial.readBytesUntil(10, serialBuffer, 100); // grab the whole message until end of line
    char *restOfMessage = serialBuffer;
    char *msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // grab everything until a space

    if (String(msgElem) == "start") {
      digitalWrite(pwmpin, pwmval); // enable it
      Serial.println("motor starting");
    }
    else if (String(msgElem) == "stop") {
      digitalWrite(pwmpin, 0); // enable it
      Serial.println("motor stopping");
    }
    else if (String(msgElem) == "fwd") {
      digitalWrite(dirpin, LOW); // enable it
      Serial.println("motor going fwd");
    }
    else if (String(msgElem) == "back") {
      digitalWrite(dirpin, HIGH); // enable it
      Serial.println("motor going back");
    }
    else if (String(msgElem) == "speed") {
      char *msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // grab everything until a space
      if (atoi(msgElem) == 0) {
        Serial.println("stepper duration not changing");
      }
      if (atoi(msgElem) > 255) {
        Serial.println("invalid speed");
      }
      else {
        pwmval = atoi(msgElem);
        Serial.print("new pwm is ");
        Serial.println(pwmval);
      }
    }
    else if (String(msgElem) == "help") {
      printCommands();
    }
    else {
      Serial.println("unknown message, please try again");
    }

    memset(serialBuffer, 0, 100); // clear the buffer
  }

  if (timer > 300) {
    Serial.println(encoderCount);
    revolutions = (float)encoderCount / countablesPerRev;
    revolutions = 360.0 * gearAngle;
    Serial.println(revolutions, 4);
    Serial.println(gearAngle, 4);
    timer = 0;
  }

}
