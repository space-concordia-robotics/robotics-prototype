
int Drill = 5;
int Drill_direction = 3;
int Elevator = 9;
int Elevator_direction = 6;
int Table = 11;
int Table_direction = 10;

void setup() {
  
  Serial.begin(9600);

  pinMode(Drill,OUTPUT);
  pinMode(Drill_direction,OUTPUT);
  pinMode(Elevator,OUTPUT);
  pinMode(Elevator_direction,OUTPUT);
  pinMode(Table,OUTPUT);
  pinMode(Table_direction,OUTPUT);
  delay(100);
  analogWrite(Drill,0);
  analogWrite(Elevator,0);
  analogWrite(Table,0);
  delay(1000);
  
  Serial.print("\nsetup complete");
  
}

void loop() {

  if(Serial.available()){
    char cmd = Serial.read();

    if(cmd=='a'){
      //turns drill counter-clockwise
      analogWrite(Drill,0);
      delay(500);
      digitalWrite(Drill_direction,HIGH);
      analogWrite(Drill,125);
      
    }
    if(cmd=='d'){
      //turns drill clockwise
      analogWrite(Drill,0);
      delay(500);
      digitalWrite(Drill_direction,LOW);
      analogWrite(Drill,125);
      
    }
    if(cmd=='s'){
      //stops drill
      analogWrite(Drill,0);
    }
    if(cmd=='e'){
       //turns elevator clockwise
      analogWrite(Elevator,0);
      delay(500);
      digitalWrite(Elevator_direction,HIGH);
      analogWrite(Elevator,150);
      
    }
    if(cmd=='q'){
      //turns elevator counter-clockwise
      analogWrite(Elevator,0);
      delay(500);
      digitalWrite(Elevator_direction,LOW);
      analogWrite(Elevator,150);
      
    }
    if(cmd=='w'){
      //stops elevator
      analogWrite(Elevator,0);
    }
    if(cmd=='l'){
       //turns table clockwise
      analogWrite(Table,0);
      delay(500);
      digitalWrite(Table_direction,HIGH);
      analogWrite(Table,245);
      
    }
    if(cmd=='j'){
      //turns table counter-clockwise
      analogWrite(Table,0);
      delay(500);
      digitalWrite(Table_direction,LOW);
      analogWrite(Table,125);
      
    }
    if(cmd=='k'){
      //stops Elevator
      analogWrite(Table,0);
    }
  }
  
}
