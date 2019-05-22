#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define TABLE_SWITCH_PIN  21
#define TRIGGER_DELAY     50

volatile bool isTriggered = false;
volatile bool isContacted = false;
bool isActualPress = false;
bool isPushed = false;
unsigned long triggerTime;


void debouncing (void);
void limSwitchTable(void);

void setup() {

  Serial.begin(115200); Serial.setTimeout(10);
  pinMode(TABLE_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TABLE_SWITCH_PIN), limSwitchTable, CHANGE);

  delay(1000);

  Serial.print("\nsetup complete");
}

void loop() {
}

void limSwitchTable(void) {
  // if this is the first time the switch was pressed in a while,
  // alert loop() that the switch was pressed and set up the timer
  if (!isTriggered && !isContacted) {
    isTriggered = true;
    triggerTime = millis();
  }

  /* every interrupt, update isContacted based on the pin state */
  // if the contact is connected, the pin will read low so set isContacted to true
  if (digitalRead(TABLE_SWITCH_PIN) == LOW) {
    isContacted = true;
    Serial.println("minus");
  }
  // otherwise the contact is bouncing so set it to false
  else if (digitalRead(TABLE_SWITCH_PIN) == HIGH) {
    isContacted = false;
    Serial.println("plus");
  }
  debouncing ();
}

void debouncing (void) {
  for (int i=0;i<=100;i++) {
    // this works but doesn't consider the possibility of both switches
    // being triggered at the same time which shouldn't ever actually happen
    if (isTriggered) {
      if ( (millis() - triggerTime) >= TRIGGER_DELAY) {
        // if the last interrupt was a press (meaning it's stabilized and in contact)
        // then there's a real press
        if (isContacted) {
          isActualPress = true;
          isPushed = isContacted;
        }
        // otherwise it's not a real press
        // so the limit switch state should stay whatever it used to be
        // and so should actualPress
        else {
          ;
        }
        // either way, we should reset the triggered bool in wait for the next trigger
        isTriggered = false;
      }
//      if (isActualPress == false && ((millis() - triggerTime) >= TRIGGER_DELAY)) break;
    }
    if (isActualPress) {
      unsigned long timer = millis();
      //Serial.println(isPushed);
      Serial.println("\nRISING");
      if (isPushed) {
        while ((millis() - timer) < 1000) {
          ;
        }
      }
      // now that the behaviour is complete we can reset these in wait for the next trigger to be confirmed
      isActualPress = false;
      isPushed = false;
//      break;
    }
  }
}

