
/*
 * Load Settings
 */



// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);


void load_settings() 
{
   pinMode(MultiSense, INPUT); // Feedback from output current
   pinMode(SEn, OUTPUT); // Enables MultiSense pin

   // Motor setting
   pinMode(MOTOR_in, OUTPUT);  
   myStepper.setSpeed(5); // Set the speed to 5 rpm
}


void enable_multisense()
{
    digitalWrite(SEn, HIGH);
}


void disable_multisense()
{
    digitalWrite(SEn, LOW);
}


void enabla_motor()
{
    digitalWrite(MOTOR_in, HIGH);
    Serial.println("Motor is Enabled!");
    myStepper.step(stepsPerRevolution); // clockwise rotation  
}


void disable_motor()
{
    digitalWrite(MOTOR_in, LOW); 
    Serial.println("Motor is Disabled!");
}


float load_voltage()
{
    int load = analogRead(MultiSense);
    float Vout = ((float)load / 1023.0) * 5;
    Serial.print("Load Value before conversion: ");
    Serial.println(load);
    
    return Vout;
}
