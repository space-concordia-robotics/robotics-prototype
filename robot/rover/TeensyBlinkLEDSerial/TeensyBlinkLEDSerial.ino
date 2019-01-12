// WARNING: when using RX/TX comms between odroid/teensy,
// make sure that you are converting the 3.3V from Teensy TX
// to 1.8V for the Odroid RX, otherwise you risk damaging the odroid RX pin
// The teensy is more tolerant so you don't necessarily need to
// convert the logic level from 1.8 to 3.3, although we still will as a redundancy
int LED_PIN = 2;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Serial USB
  Serial.begin(9600);
  // Serial UART
  Serial1.begin(9600);
}

void loop() {
    // only respond to UART serial messages (not USB)
    if (Serial1.available()) {
        String cmd = Serial1.readString().trim();
        Serial.print("cmd: ");
        Serial.println(cmd);

        if (cmd.equals("1")) {
            digitalWrite(LED_PIN, HIGH);
            //Serial1.println("HIGH");
            Serial.println("HIGH");
        }
        else if (cmd.equals("0")) {
            digitalWrite(LED_PIN, LOW);
            //Serial1.println("LOW");
            Serial.println("LOW");
        }
    }
}
