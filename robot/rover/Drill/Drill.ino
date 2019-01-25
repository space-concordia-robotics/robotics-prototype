int Drill = 5;
int Drill_direction = 3;
int Pump = 9;
int Pump_direction = 6;

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

    if(cmd=='j'){
      
      digitalWrite(Drill_direction,HIGH);
      analogWrite(Drill,100);
      delay(10000);
      analogWrite(Drill,0);
    }
    if(cmd=='l'){
      
      digitalWrite(Drill_direction,LOW);
      analogWrite(Drill,100);
      delay(10000);
      analogWrite(Drill,0);
    }
    if(cmd=='u'){
      
      digitalWrite(Pump_direction,HIGH);
      analogWrite(Pump,100);
      delay(10000);
      analogWrite(Pump,0);
    }
    if(cmd=='o'){
      
      digitalWrite(Pump_direction,LOW);
      analogWrite(Pump,100);
      delay(10000);
      analogWrite(Pump,0);
    }
    if(cmd=='a'){
      //turns drill counter-clockwise
      digitalWrite(Drill_direction,HIGH);
      analogWrite(Drill,100);
      
    }
    if(cmd=='d'){
      //turns drill clockwise
      digitalWrite(Drill_direction,LOW);
      analogWrite(Drill,100);
      
    }
    if(cmd=='s'){
      //stops drill
      analogWrite(Drill,0);
    }
    if(cmd=='e'){
       //turns pump clockwise
      digitalWrite(Pump_direction,HIGH);
      analogWrite(Pump,100);
      
    }
    if(cmd=='q'){
      //turns pump counter-clockwise
      digitalWrite(Pump_direction,LOW);
      analogWrite(Pump,100);
      
    }
    if(cmd=='w'){
      //stops pump
      analogWrite(Pump,0);
    }
    
  }
  
}

