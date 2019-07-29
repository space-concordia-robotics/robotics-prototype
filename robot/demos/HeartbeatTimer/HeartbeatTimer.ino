unsigned long int lastTime = 0;
const int led = 13;
bool ledState = LOW;
const long delayInterval = 1000;
const long stateInterval = 250;

void setup() {
  pinMode(led, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  //place code to loop here
  heartbeat();
}

void goodBlink(){
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= 1000){
    previousMillis = currentMillis;
    ledSwap();
    digitalWrite(led, ledState);
    Serial.println(ledState);
  }
}

void ledSwap(){
  if(ledState == HIGH){
    ledState = LOW;
  } else{
    ledState = HIGH;
  }
}
