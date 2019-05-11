unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 500;           // interval at which to change from high to low or vice-versa (milliseconds)
const int led = 13;      // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED
bool state = false;

/*
 * Type 1 for LED blink
 * 
 * Type 0 for LED off
 */
void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    Serial.print("I received: ");
    Serial.println(inByte);
    switch (inByte) {
      case 48:
        digitalWrite(led, HIGH);
        Serial.println("Test1");
        state = false;
        break;
      case 49:
        digitalWrite(led, LOW);
        Serial.println("Test2");
        state = true;
        break;
    }
    Serial.println(state);
  }
  if (state == true) {
    Blink();
  } else {
    digitalWrite(led, LOW); // check to make sure it's off incase it was still on
  }
}

void Blink() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(led, ledState);
    Serial.println("Test3");
  }
}
