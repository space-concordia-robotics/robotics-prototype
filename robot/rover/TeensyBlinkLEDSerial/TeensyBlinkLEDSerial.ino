// For testing used a single red LED with a 200 Ohm resistor
// to get approximately 15 mA with teensys 3.3V

// Refer to pinout for teensy 3.6
int LED_PIN = 2;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == '1') {
            digitalWrite(LED_PIN, HIGH);
            Serial.println("HIGH");
        }
        else if (cmd == '0') {
            digitalWrite(LED_PIN, LOW);
            Serial.println("LOW");
        }
    }
}
