void setup() {
  Serial.begin(57600);
  Serial.println("setting up");
}

void loop() {
  for (int i = 0; i < 100; i++) {
     String str = "loop #";
     str.concat(i);
     Serial.println(str);
     delay(777); // ms
  }
}
