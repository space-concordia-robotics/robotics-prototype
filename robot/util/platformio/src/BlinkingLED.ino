int incomingByte;
int LONG_PAUSE = 200;
int SHORT_PAUSE = 100;
// the setup function runs once when you press reset or power the board
void setup() {
    // open serial port, set data rate to 9600
    Serial.begin(9600);
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
    // do things only when you receive data

    if (Serial.available() > 0) {
        // read the incoming byte
        incomingByte = Serial.read();

        if (incomingByte == 'w') {
            for (int i = 0; i < 3; i ++) {
                digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
                delay(SHORT_PAUSE);                // wait for a bit
                digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
                delay(SHORT_PAUSE);                // wait for a bit
            }
        }
        else if (incomingByte == 's') {
            for (int i = 0; i < 5; i++) {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(LONG_PAUSE);
                digitalWrite(LED_BUILTIN, LOW);
                delay(LONG_PAUSE);
            }
        }
    }
}
