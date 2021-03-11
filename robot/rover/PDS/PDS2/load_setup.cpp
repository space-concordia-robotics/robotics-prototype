#include "load_setup.h"

// Create stepper object called 'myStepper', note the pin order:


Setup::Setup()
{
   pinMode(multiSense, INPUT); // Feedback from output current
   pinMode(SEn, OUTPUT); // Enables MultiSense pin

   // Motor setting
   pinMode(motor_input, OUTPUT);  
   //myStepper.setSpeed(5); // Set the speed to 5 rpm  
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
    PORTB |= 0B00000001; //motor one
    PORTD |= 0B11000000; //motor two and three
    PORTC |= 0B00111000; //motor four, five and six  
    Serial.println("All Motors are Enabled!");
}

void Setup::disable_all_motors()
{
    PORTB &= 0B11111110; //motor one
    PORTD &= 0B00111111; //motor two and three
    PORTC &= 0B11000111; //motor four, five and six  
    Serial.println("All Motors are Disabled!");  
}


int Setup::get_status(int pin1)
{
    return digitalRead(pin1);
}

void Setup::get_status_motors()
{
    for (int i = 0; i < NUM_MOTORS; i++) 
    {
        motorState[i] = digitalRead(motorPins[i]);
    }    
}


float Setup::load_voltage()
{
    int load = analogRead(multiSense);
    float Vout = ((float)load / 1023.0) * 5;
//    Serial.print("Load Value before conversion: ");
//    Serial.println(load);
    
    return Vout;
}
