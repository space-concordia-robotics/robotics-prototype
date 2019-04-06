unsigned long int lastTime = 0;
const int led = 13;
bool ledState = LOW;
const long delayInterval = 1000;
const long stateInterval = 250;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
}

void loop() {
  unsigned long int currentMillis = millis();
  
  if(currentMillis - lastTime > delayInterval){
    ledState = !(ledState);
    digitalWrite(led, ledState);
    lastTime = currentMillis;
  } //if
  else if(currentMillis - lastTime > stateInterval){
    ledState = !(ledState);
    digitalWrite(led, ledState);
  } //elseif
} //loop

//To add a 2nd led which follows its own timing, just add a "lastTime2", and the const time values for the 2nd LED into
//an extra if-else-if statement like the one above.
