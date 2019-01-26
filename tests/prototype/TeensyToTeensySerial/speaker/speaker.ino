int rest = 200;
elapsedMillis dt;
char cmds[] = {'1', '2', '3', '4'};
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
    //Serial1.print("one");
    Serial1.write(cmds[i++]);
    //Serial1.flush();
    if (i == 4) {
      i = 0;
    }
  }
//  if (dt > 4000 && dt < 4100) {
//    dt = 0;
//    Serial.println("boop");
//    //Serial1.print("two");
//    Serial1.write('2');
//    Serial1.flush();
//  }
  /*Serial.print('a');
  delay(2000);
  Serial1.write('b');
  Serial1.flush();
  Serial.print('b');
  delay(2000);
  Serial1.write('c');
  Serial1.flush();
  Serial.print('c');
  delay(2000);
  Serial1.write('d');
  Serial1.flush();
  Serial.print('d');
  delay(2000);*/
}
