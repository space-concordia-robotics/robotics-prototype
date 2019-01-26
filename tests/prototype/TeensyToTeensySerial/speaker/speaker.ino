int rest = 200;
elapsedMillis dt;
String cmds[] = {"one", "two", "three", "four"};
int i = 0;

void blinkLed(int halfTime) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(halfTime);
  digitalWrite(LED_BUILTIN, LOW);
  delay(halfTime);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // startup routine
  for (int i = 0; i < 3; i++) {
    blinkLed(rest);
  }
  Serial.begin(9600);
  Serial1.begin(9600);
  //dt = 0;
}

void loop() {
  if (dt > 2000) {
    dt = 0;
    Serial.println("beep");
    Serial1.print(cmds[i++]);
    //Serial1.write(cmds[i++]);
    //Serial1.flush();
    if (i == 4) {
      i = 0;
    }
  }
}
