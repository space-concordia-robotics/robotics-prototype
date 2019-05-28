int dogTimer = 0;
unsigned long int previousMillis = 0;
const int led = 13;
bool ledState = LOW;
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  setDogTime(500);
  kickDog();
  
}

void loop() {
  unsigned long int currentTime = millis();
  while (true) {
    Blink();
    if((currentTime - dogTimer) > 10){ 
     kickDog();
     dogTimer = millis();
    } //if
  } //while
} //loop

//The function "kicks the dog". Refreshes its timeout counter. If not refreshed, system will be reset.
void kickDog(){
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}
void Blink(){
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= 100){
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(led, ledState);
    Serial.println("test");
  }
}
void setDogTime(int duration){
  
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1; 
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2; 
  WDOG_TOVALL = duration; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
  WDOG_TOVALH = 0; //End value WDT compares itself to. 
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | 
  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
  WDOG_PRESC = 0; //Sets watchdog timer to tick at 1 kHz inseast of 1/4 kHz
}
