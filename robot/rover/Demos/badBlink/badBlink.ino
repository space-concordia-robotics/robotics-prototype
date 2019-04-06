unsigned long int previousMillis = 0;
const int led = 13;
bool ledState = LOW;
bool complete = false;
int counter = 0;
void setup() {
  pinMode(led, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
 //place code to loop here
  if(complete == false){
    goodBlink();
  }
  else{
    digitalWrite(led, LOW);
    Serial.println("2 blinks complete");
  }
}

void goodBlink(){
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= 1000){ //set to 100 for badblink, set to 250 for goodblink
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(led, ledState);
    Serial.println(ledState);
    counter++;
  }
  //if(counter == 4){ //set to 12 for badblink. 4 for goodblink
  //  complete = true;
  //}
}
