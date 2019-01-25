

int Drill = 9;
int Drill_direction = 3;
int Pump = 5;
int Pump_direction = 7;

void setup() {
  
	Serial.begin(9600);

  pinMode(Drill,OUTPUT);
  pinMode(Drill_direction,OUTPUT);
  pinMode(Pump,OUTPUT);
  pinMode(Pump_direction,OUTPUT);
  delay(1000);
  
  Serial.print("\nsetup complete");
  
}

void loop() {

  if(Serial.available()){
    char cmd = Serial.read();

    if(cmd=='w'){
      
      digitalWrite(Drill_direction,HIGH);
      analogWrite(Drill,100);
      delay(500);
      analogWrite(Drill,0);
    }
    if(cmd=='s'){
      
      digitalWrite(Drill_direction,LOW);
      analogWrite(Drill,100);
      delay(500);
      analogWrite(Drill,0);
    }
    if(cmd=='r'){
      
      digitalWrite(Pump_direction,HIGH);
      analogWrite(Pump,100);
      delay(500);
      analogWrite(Pump,0);
    }
    if(cmd=='f'){
      
      digitalWrite(Pump_direction,LOW);
      analogWrite(Pump,100);
      delay(500);
      analogWrite(Pump,0);
    }
  }
  
}

