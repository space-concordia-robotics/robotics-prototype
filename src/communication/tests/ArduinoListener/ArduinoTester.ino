void setup() {
  Serial.begin(9600);
  
  /*This is the check condition to connect at the proper baudrate*/
  Serial.println("tester!");
  }

void loop() {
  
  for (int i = 0; i < 100; i++) {
     String str = "loop #";
     str.concat(i);
     Serial.println(str);
     delay(1000); // ms
  }
  delay(1000);
}

