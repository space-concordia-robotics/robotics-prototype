// WARNING: when using RX/TX comms between odroid/teensy,
// make sure that you are converting the 3.3V from Teensy TX
// to 1.8V for the Odroid RX, otherwise you risk damaging the odroid RX pin
// The teensy is more tolerant so you don't necessarily need to
// convert the logic level from 1.8 to 3.3, although we still will as a redundancy

int LED_DEV_0 = 2;
int LED_DEV_1 = 3;
int LED_DEV_2 = 4;
int LED_DEV_3 = 5;

void setup() {
    pinMode(LED_DEV_0, OUTPUT);
    pinMode(LED_DEV_1, OUTPUT);
    pinMode(LED_DEV_2, OUTPUT);
    pinMode(LED_DEV_3, OUTPUT);

    // to show a sign of life
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);

    // To combat latency, try higher baudrates like
    // 19200, 38400, 57600, 74880, 115200
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
            digitalWrite(LED_DEV_0, HIGH);
            //Serial1.println("HIGH");
            Serial.println("HIGH");
        }
        else if (cmd.equals("0")) {
            digitalWrite(LED_DEV_0, LOW);
            //Serial1.println("LOW");
            Serial.println("LOW");
        }
    }
    if (Serial2.available()) {
        String cmd = Serial1.readString().trim();
        Serial.print("cmd: ");
        Serial.println(cmd);

        if (cmd.equals("1")) {
            digitalWrite(LED_DEV_1, HIGH);
            //Serial1.println("HIGH");
            Serial.println("HIGH");
        }
        else if (cmd.equals("0")) {
            digitalWrite(LED_DEV_1, LOW);
            //Serial1.println("LOW");
            Serial.println("LOW");
        }
    }
    if (Serial3.available()) {
        String cmd = Serial1.readString().trim();
        Serial.print("cmd: ");
        Serial.println(cmd);

        if (cmd.equals("1")) {
            digitalWrite(LED_DEV_2, HIGH);
            //Serial1.println("HIGH");
            Serial.println("HIGH");
        }
        else if (cmd.equals("0")) {
            digitalWrite(LED_DEV_2, LOW);
            //Serial1.println("LOW");
            Serial.println("LOW");
        }
    }
    if (Serial4.available()) {
        String cmd = Serial1.readString().trim();
        Serial.print("cmd: ");
        Serial.println(cmd);

        if (cmd.equals("1")) {
            digitalWrite(LED_DEV_3, HIGH);
            //Serial1.println("HIGH");
            Serial.println("HIGH");
        }
        else if (cmd.equals("0")) {
            digitalWrite(LED_DEV_3, LOW);
            //Serial1.println("LOW");
            Serial.println("LOW");
        }
    } else if (Serial.available()) {
        String cmd = Serial.readString().trim();
        Serial.print("cmd: ");
        Serial.println(cmd);

        if (cmd.equals("1")) {
            digitalWrite(LED_BUILTIN, HIGH);
            //Serial1.println("HIGH");
            Serial.println("HIGH");
        }
        else if (cmd.equals("0")) {
            digitalWrite(LED_BUILTIN, LOW);
            //Serial1.println("LOW");
            Serial.println("LOW");
        }
    }
}
