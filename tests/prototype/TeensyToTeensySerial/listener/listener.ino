int rest = 150;

void blinkLed(int halfTime) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(halfTime);
  digitalWrite(LED_BUILTIN, LOW);
  delay(halfTime);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // startup routine: blink 5 times
  for (int i = 0; i < 5; i++) {
    blinkLed(rest);
  }

  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  if (Serial1.available()) {
    Serial.println("received data!");
    //String cmd = Serial.readString().trim();
    char cmd = Serial1.read();
    Serial.print("cmd: ");
    Serial.println(cmd);

    //    if (cmd.equals("one")) {
    //      Serial.println("1");
    //      blinkLed(rest);
    //    } else if (cmd.equals("two")) {
    //      Serial.println("2");
    //      blinkLed(rest);
    //      blinkLed(rest);
    //    } else if (cmd.equals("three")) {
    //      Serial.println("3");
    //      for (int i = 0; i < 3; i++) {
    //        blinkLed(rest);
    //      }
    //    } else if (cmd.equals("four")) {
    //      Serial.println("4");
    //      for (int i = 0; i < 4; i++) {
    //        blinkLed(rest);
    //      }
    //    }

    switch (cmd) {
      case '1':
        Serial.println("1");
        blinkLed(rest);
        break;
      case '2':
        Serial.println("2");
        blinkLed(rest);
        blinkLed(rest);
        break;
      case '3':
        Serial.println("3");
        for (int i = 0; i < 3; i++) {
          blinkLed(rest);
        }
        break;
        case '4':
          Serial.println("4");
          for (int i = 0; i < 4; i++) {
            blinkLed(rest);
          }
        break;
       default:
        Serial.println("default");
        break;
    }

  }
}
