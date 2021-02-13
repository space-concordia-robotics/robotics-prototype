#include "load_setup.h"

// Create stepper object called 'myStepper', note the pin order:


Setup::Setup()
{
   pinMode(MultiSense, INPUT); // Feedback from output current
   pinMode(SEn, OUTPUT); // Enables MultiSense pin

   // Motor setting
   pinMode(MOTOR_in, OUTPUT);  
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


int Setup::get_status(int pin1)
{
    return digitalRead(pin1);
}


float Setup::load_voltage()
{
    int load = analogRead(MultiSense);
    float Vout = ((float)load / 1023.0) * 5;
    Serial.print("Load Value before conversion: ");
    Serial.println(load);
    
    return Vout;
}


/*void Setup::load_settings() 
{
   pinMode(MultiSense, INPUT); // Feedback from output current
   pinMode(SEn, OUTPUT); // Enables MultiSense pin

   // Motor setting
   pinMode(MOTOR_in, OUTPUT);  
   //myStepper.setSpeed(5); // Set the speed to 5 rpm  
}*/

/*
void Setup::enable_multisense()
{
    digitalWrite(SEn, HIGH);
}


void Setup::disable_multisense()
{
    digitalWrite(SEn, LOW);
}


void Setup::enable_motor()
{
    digitalWrite(MOTOR_in, HIGH);
    Serial.println("Motor is Enabled!");
    //myStepper.step(stepsPerRevolution); // clockwise rotation  
}


void Setup::disable_motor()
{
    digitalWrite(MOTOR_in, LOW); 
    Serial.println("Motor is Disabled!");
}*/


/*
void Setup::enable_fans()
{
    digitalWrite(Fans, HIGH);
    Serial.println("Fans are Enabled!");  
}


void disable_fans()
{
    digitalWrite(Fans, LOW);
    Serial.println("Fans are Disabled!");  
}*/
