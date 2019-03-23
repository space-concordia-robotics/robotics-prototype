int CurrentTime;
int DogTimer = 0;
const int led = 13;
bool ledState = false;
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  // Indicate we are starting over by hold led off for 1s
  ledState = false;
  digitalWrite(led, ledState);
  delay(1000);
  // Indicate we are in setup by hold LED on
  ledState = true;
  digitalWrite(led, ledState);
  delay(2000);
  setDogTime(500);
  kickDog();
}

void loop() {
  
  CurrentTime = millis();
  while (true) {
    ledState = !ledState;
    digitalWrite(led, ledState);
    delay(100);
  if((CurrentTime - DogTimer) > 10){ 
    kickDog();
    DogTimer = millis();
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

void setDogTime(int duration){
  
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1; 
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2; 
  WDOG_TOVALL = duration; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
  WDOG_TOVALH = 0; //End value WDT compares itself to. 
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | 
  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
  WDOG_PRESC = 0; //Sets watchdog timer to tick at 1 kHz inseast of 1/4 kHz
}
