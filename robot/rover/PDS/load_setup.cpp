#include "load_setup.h"

// Create stepper object called 'myStepper', note the pin order:


Setup::Setup()
{
   int load_enable_1 = 13;
   int load_enable_2 = 12;
   int load_enable_3 = 11;
   int load_enable_4 = 9;
   int load_enable_5 = 6;
   int load_enable_6 = A5;
   int Fan_EN = 5;

   int Sense_read_1=A1;
   int Sense_read_2=A0;
   int Sense_read_3=A7;
   int Sense_read_4=A4;
   int Sense_read_5=A3;
   int Sense_read_6=A2;
   
     
   pinMode(Fan_EN,OUTPUT); // Defines Fan_EN output ,
   // Load switches pins
   pinMode(load_enable_1, OUTPUT);  // Load enable1 
   pinMode(load_enable_2, OUTPUT);  // Load enable2
   pinMode(load_enable_3, OUTPUT);  // Load enable3
   pinMode(load_enable_4, OUTPUT);  // Load enable4
   pinMode(load_enable_5, OUTPUT);  // Load enable5
   pinMode(load_enable_6, OUTPUT);  // Load enable6

   // Current reading pins

   pinMode(Sense_read_1,INPUT); // Sense read1
   pinMode(Sense_read_2,INPUT); // Sense read2
   pinMode(Sense_read_3,INPUT); // Sense read3
   pinMode(Sense_read_4,INPUT); // Sense read4
   pinMode(Sense_read_5,INPUT); // Sense read5
   pinMode(Sense_read_6,INPUT); // Sense read6
}


void Setup::enable_device(String message, int pin1)
{
   digitalWrite(pin1, HIGH);
   Serial.println(message);
}


void Setup::disable_device(String message, int pin1)
{
    digitalWrite(pin1, LOW);  
    Serial.println(message);
}

void Setup::enable_all_motors()
{
    PORTB |= 0B00111010; //load switch 1,2,3,4 
    PORTD |= 0B01000000; //load switch 5
    PORTC |= 0B00100000; //load switch 6  
    Serial.println("All Motors are Enabled!");
}

void Setup::disable_all_motors()
{
    PORTB &= 0B11000101; //load switch 1,2,4,6 
    PORTD &= 0B10111111; //load switch 3
    PORTC &= 0B11011111; //load switch 5 
    Serial.println("All Motors are Disabled!");  
}


int Setup::get_status(int pin1)
{
    return digitalRead(pin1);
}

void Setup::get_status_motors()  //not returning? just reading?
{
    for (int i = 0; i < 6; i++) 
    {
        motorState[i] = digitalRead(motorPins[i]);
    }    
}


float Setup::load_current(int load_switch)
{
  int load_switch_read;
  
    if (load_switch == 1)
          load_switch_read = A1;
    else if (load_switch == 2)
            load_switch_read = A0;
    else if (load_switch == 3)
            load_switch_read = A7;
    else if (load_switch == 4)
            load_switch_read = A4;
    else if (load_switch == 5)
            load_switch_read = A3;
    else if (load_switch == 6)
            load_switch_read = A2;
    else
    {}
    float sense=0;
    for (int i=0;i<10;i++)
    {
      sense+=analogRead(load_switch_read);
      delayMicroseconds(10);
    }
    float constant=lookup(sense/10);
    float current = (sense/10)*constant; 
    Serial.print("Current in the motor_");
    Serial.print(load_switch);
    Serial.print(" ");
    Serial.print(current);
    Serial.println(" A");
    return current;
}

float Setup::lookup(float value)
{
  float K;
  if (value<=9.94)
    K=0.0222;
  else if (value>9.94 && value<=53.92)
  K=0.01752;
  else if(value>53.91 && value<=68.25) 
  K=0.016299;
  else if(value>68.25 && value<=129.94)
  K=0.01539752;
  else if(value>129.94 && value<=140.83)
  K=0.01494371;
  else if(value>140.83 && value<=151.8)
  K=0.014857358;
  else if(value>151.8 && value<=170)
  K=0.014751478;
  else if(value>170 && value<206.75)
  K=0.014600785;
  else if(value>206.75)
  K=0.01432109; // constant 0.01432109 derived from schematic and datasheet of VN5050
  else
  {}
  
  return K;
  
  }
