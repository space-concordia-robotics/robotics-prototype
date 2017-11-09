void setup() {
  Serial.begin(57600);
  Serial.println("setting up (PETER IS LITERALLY A DEMON FROM HELL)");
}

void loop() {
  for (int i = 0; i < 100; i++) {
     String str = "loop #";
     str.concat(i);
     Serial.println(str);
     delay(777); // ms
  }
}
