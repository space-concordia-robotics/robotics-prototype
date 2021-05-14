#include "load_setup.h"

// Create stepper object called 'myStepper', note the pin order:


Setup::Setup()
{
   pinMode(9,OUTPUT); // Defines Fan_EN output ,

   // Load switches pins
   pinMode(15, OUTPUT);  // Load enable1 
   pinMode(16, OUTPUT);  // Load enable2
   pinMode(2, OUTPUT);   // Load enable3
   pinMode(17, OUTPUT);  // Load enable4
   pinMode(28, OUTPUT);  // Load enable5
   pinMode(13, OUTPUT);  // Load enable6

   // Current reading pins

   pinMode(27,INPUT); // Sense read1
   pinMode(26,INPUT); // Sense read2
   pinMode(25,INPUT); // Sense read3
   pinMode(24,INPUT); // Sense read4
   pinMode(23,INPUT); // Sense read5
   pinMode(22,INPUT); // Sense read6
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
    PORTB |= 0B00111010; //load switch 1,2,4,6 
    PORTD |= 0B00010000; //load switch 3
    PORTC |= 0B00100000; //load switch 5  
    Serial.println("All Motors are Enabled!");
}

void Setup::disable_all_motors()
{
    PORTB &= 0B11000101; //load switch 1,2,4,6 
    PORTD &= 0B11101111; //load switch 3
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
    float current = (analogRead(28-load_switch)*0.01432109); // constant 0.01432109 derived from schematic and datasheet of vn5050
    
    Serial.print("Current in the motor_");
    Serial.print(load_switch);
    Serial.print(" ");
    Serial.print(current);
    Serial.println(" A");
    return current;
}
